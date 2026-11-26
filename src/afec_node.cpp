#include "afec_node.h"
#include <string.h>

// ------------------------------------------------------------------ logging
// printDebug is printf-style: void (*printDebug)(const char*, ...)
// Compile-time verbosity: 0=off 1=ERR 2=WARN 3=INFO 4=DBG  (override with -DAFEC_LOG_LEVEL=n)
// Compile-time format:    -DAFEC_LOG_FORMAT=0  human text (default):  "[millis] N<id> <L> message"
//                         -DAFEC_LOG_FORMAT=1  JSON Lines, one object per line:
//                              {"t":<millis>,"node":<id>,"lv":"E|W|I|D","ev":"<event>",...fields}
//                         Field/event reference: see afec_log_schema.md. Both formats log the same
//                         events at the same levels; JSON values are plain numbers or short ASCII strings
//                         (no escaping needed). Filter a mixed serial capture with: lines starting with '{'.
// Only valid inside Node members.
#ifndef AFEC_LOG_LEVEL
#define AFEC_LOG_LEVEL 3
#endif
#ifndef AFEC_LOG_FORMAT
#define AFEC_LOG_FORMAT 0
#endif

// AFEC_LOG(level-expr, event, text-format, json-fields, args...)  -- text and JSON share ONE argument list
#if AFEC_LOG_FORMAT == 1
#define AFEC_LOG(lvl, ev, tfmt, jfmt, ...)                                                \
    do {                                                                                  \
        if ((lvl) <= AFEC_LOG_LEVEL && printDebug)                                        \
            printDebug("{\"t\":%lu,\"node\":%u,\"lv\":\"%c\",\"ev\":\"" ev "\"" jfmt "}\n", \
                       (unsigned long)_p.millis(), (unsigned)_cfg.id, "EWID"[(lvl) - 1],  \
                       ##__VA_ARGS__);                                                    \
    } while (0)
#else
#define AFEC_LOG(lvl, ev, tfmt, jfmt, ...)                                                \
    do {                                                                                  \
        if ((lvl) <= AFEC_LOG_LEVEL && printDebug)                                        \
            printDebug("[%lu] N%u %c " tfmt "\n", (unsigned long)_p.millis(),             \
                       (unsigned)_cfg.id, "EWID"[(lvl) - 1], ##__VA_ARGS__);              \
    } while (0)
#endif

#define LOGE(ev, t, j, ...) AFEC_LOG(1, ev, t, j, ##__VA_ARGS__)
#define LOGW(ev, t, j, ...) AFEC_LOG(2, ev, t, j, ##__VA_ARGS__)
#define LOGI(ev, t, j, ...) AFEC_LOG(3, ev, t, j, ##__VA_ARGS__)
#define LOGD(ev, t, j, ...) AFEC_LOG(4, ev, t, j, ##__VA_ARGS__)
#define LOGL(lvl, ev, t, j, ...) AFEC_LOG(lvl, ev, t, j, ##__VA_ARGS__)   // level chosen at run time (1..4)

namespace afec {

namespace {

[[maybe_unused]] const char* resultStr(Result r) {
    switch (r) {
        case Result::OK:         return "OK";
        case Result::TOO_LONG:   return "TOO_LONG";
        case Result::DUTY_LIMIT: return "DUTY_LIMIT";
        case Result::RADIO_ERR:  return "RADIO_ERR";
        case Result::NO_ACK:     return "NO_ACK";
        default:                 return "?";
    }
}

[[maybe_unused]] const char* typeStr(MsgType t) {
    switch (t) {
        case MsgType::DATA: return "DATA";
        case MsgType::ACK:  return "ACK";
        case MsgType::NACK: return "NACK";
        default:            return "?";
    }
}

[[maybe_unused]] const char* actionStr(Action a) {
    switch (a) {
        case Action::DOWNGRADE:    return "DOWNGRADE";
        case Action::HOLD:         return "HOLD";
        case Action::UPGRADE:      return "UPGRADE";
        case Action::NACK_UPGRADE: return "NACK_UPGRADE";
        case Action::MAX_PROTECT:  return "MAX_PROTECT";
        default:                   return "?";
    }
}

} // namespace

bool Node::begin(CC1101& radio, const NodeConfig& cfg, const Platform& plat) {
    if (!plat.millis || !plat.random) return false;
    _r = &radio; _cfg = cfg; _p = plat;
    if (_cfg.initialLevel > MAX_LEVEL) _cfg.initialLevel = MAX_LEVEL;
    eccInit();
    _lhs.begin(_cfg.lhs);
    _hopLevel = _cfg.initialLevel;
    _budgetUs = (int64_t)_cfg.dutyBurstMs * 1000;
    _lastRefill = _p.millis();
    LOGI("begin", "BEGIN mode=%u initLvl=%u bitrate=%lu duty=%u%% burst=%ums retries=%u ackTo=%ums", ",\"mode\":%u,\"init_lvl\":%u,\"bitrate\":%lu,\"duty_pct\":%u,\"burst_ms\":%u,\"retries\":%u,\"ack_to_ms\":%u", (unsigned)_cfg.mode, _cfg.initialLevel, (unsigned long)_cfg.bitrateBps, (unsigned)_cfg.dutyPercent, (unsigned)_cfg.dutyBurstMs, _cfg.maxRetries, (unsigned)_cfg.ackTimeoutMs);
    return true;
}

// ------------------------------------------------------------------ level selection
Node::Peer& Node::peer(uint8_t id) {
    Peer* victim = &_peers[0];
    for (auto& p : _peers) {
        if (p.used && p.id == id) { p.lastUse = ++_tick; return p; }
        if (!p.used) victim = &p;
        else if (victim->used && p.lastUse < victim->lastUse) victim = &p;
    }
    if (victim->used) LOGD("peer_evict", "PEER evict id=%u", ",\"id\":%u", victim->id);
    *victim = Peer(); victim->used = true; victim->id = id;
    victim->level = _cfg.initialLevel; victim->ackLevel = _cfg.initialLevel; victim->lastUse = ++_tick;
    LOGD("peer_new", "PEER new id=%u lvl=%u", ",\"id\":%u,\"lvl\":%u", id, victim->level);
    return *victim;
}

void Node::resetLink(uint8_t id) {
    Peer& p = peer(id); p.level = _cfg.initialLevel; p.streak = 0; p.rxFb = 0xA;
    p.ackLevel = _cfg.initialLevel; p.ackStreak = 0;
    _hopLevel = _cfg.initialLevel; _hopStreak = 0;
    LOGI("link_reset", "LINK reset peer=%u lvl=%u", ",\"peer\":%u,\"lvl\":%u", id, p.level);
}

uint8_t Node::peerLevel(uint8_t id) { return levelFor(peer(id)); }

uint8_t Node::levelFor(const Peer& p) const { return adaptive() ? p.level : LEVEL_NONE; }

uint8_t Node::rssiQ(int16_t r) const {
    const RssiPolicy& p = _cfg.rssi;
    if (r < p.minDbm) r = p.minDbm;
    if (r > p.maxDbm) r = p.maxDbm;
    return (uint8_t)(((int32_t)(r - p.minDbm) * 15) / (p.maxDbm - p.minDbm));
}

// Pure mapping, no logging here: callers log with full context (src/seq/rssi/lqi).
uint8_t Node::feedback(uint8_t score, int16_t rssi) const {
    if (_cfg.mode == Mode::AFEC) return score;
    if (_cfg.mode == Mode::RSSI) return rssiQ(rssi);
    return 0;
}

// One level step per ACK. C5: Table 1. C4: threshold + hysteresis on the same 4 bits.
// Logs one line per step: INFO if the level moved, DBG otherwise.
void Node::stepLevel(uint8_t& level, uint8_t& streak, uint8_t fb, const char* tag, uint8_t id) {
    const uint8_t prevLevel = level, prevStreak = streak;
    const char* act = "-";

    if (_cfg.mode == Mode::AFEC) {
        const Action a = Lhs::decide(fb);
        act = actionStr(a);
        switch (a) {
            case Action::DOWNGRADE:
                if (++streak >= _cfg.downgradeAfter) { if (level > 0) level--; streak = 0; }
                break;
            case Action::HOLD:         streak = 0; break;
            case Action::UPGRADE:      streak = 0; if (level < MAX_LEVEL) level++; break;
            case Action::NACK_UPGRADE: streak = 0; nackJump(level); break;
            case Action::MAX_PROTECT:  streak = 0; level = MAX_LEVEL; break;
        }
    } else if (_cfg.mode == Mode::RSSI) {
        const RssiPolicy& p = _cfg.rssi;
        if (level < MAX_LEVEL && fb < p.up[level]) { level++; act = "RSSI_UP"; }
        else if (level > 0 && fb >= p.up[level - 1] + p.hys) { level--; act = "RSSI_DOWN"; }
        else act = "RSSI_HOLD";
    }

    LOGL(level != prevLevel ? 3 : 4, "lvl_step", "LVL %s[%u] %s fb=0x%X lvl %u->%u streak %u->%u/%u",
         ",\"tag\":\"%s\",\"id\":%u,\"action\":\"%s\",\"fb\":%u,\"from\":%u,\"to\":%u,\"streak_from\":%u,\"streak_to\":%u,\"downgrade_after\":%u",
         tag, id, act, fb, prevLevel, level, prevStreak, streak, _cfg.downgradeAfter);
}

void Node::nackJump(uint8_t& level) {
    level = (level + _cfg.nackStep > MAX_LEVEL) ? MAX_LEVEL : level + _cfg.nackStep;
}

bool Node::seenOrAdd(uint8_t src, uint8_t seq) {
    for (auto& s : _seen) if (s.used && s.src == src && s.seq == seq) return true;
    _seen[_seenHead] = { src, seq, true };
    _seenHead = (_seenHead + 1) % NSEEN;
    return false;
}

// ------------------------------------------------------------------ duty cycle
uint32_t Node::airtimeUs(uint8_t bytes) const {
    uint32_t us = (uint32_t)(bytes + 9) * 8UL * 1000000UL / _cfg.bitrateBps;
    return _cfg.mode == Mode::HWFEC ? us * 2 : us;               // rate-1/2 FEC doubles airtime
}

void Node::refillDuty() {
    uint32_t now = _p.millis(), dt = now - _lastRefill; _lastRefill = now;
    _budgetUs += (int64_t)dt * 10 * _cfg.dutyPercent;
    int64_t cap = (int64_t)_cfg.dutyBurstMs * 1000;
    if (_budgetUs > cap) _budgetUs = cap;
}

// ------------------------------------------------------------------ TX path
Result Node::transmit(const Frame& fin, uint8_t level) {
    Frame f = fin;
    if (_cfg.fixedPayloadLen && f.plen < _cfg.fixedPayloadLen) {      // C3 fixed-length radio: pad
        memset(f.payload + f.plen, 0, _cfg.fixedPayloadLen - f.plen); f.plen = _cfg.fixedPayloadLen;
    }
    uint8_t buf[AIR_MAX], n = 0;
    const uint8_t reqLevel = level;
    level = fitLevel(level, f.plen);
    if (level != reqLevel)
        LOGD("tx_clamp", "TX level clamp %u->%u (plen=%u)", ",\"req_lvl\":%u,\"lvl\":%u,\"plen\":%u", reqLevel, level, f.plen);
    if (!encodeFrame(f, level, buf, &n)) {
        LOGW("tx_too_long", "TX %s %u>%u seq=%u TOO_LONG plen=%u lvl=%u", ",\"type\":\"%s\",\"src\":%u,\"dst\":%u,\"seq\":%u,\"plen\":%u,\"lvl\":%u", typeStr(f.type), f.src, f.dst, f.seq, f.plen, level);
        return Result::TOO_LONG;
    }

    refillDuty();
    uint32_t at = airtimeUs(n);
    if (_budgetUs < (int64_t)at) {
        _st.dutyBlocked++;
        LOGW("tx_duty_limit", "TX %s %u>%u seq=%u DUTY_LIMIT need=%luus have=%ldus blocked=%lu", ",\"type\":\"%s\",\"src\":%u,\"dst\":%u,\"seq\":%u,\"need_us\":%lu,\"have_us\":%ld,\"blocked\":%lu", typeStr(f.type), f.src, f.dst, f.seq, (unsigned long)at, (long)_budgetUs, (unsigned long)_st.dutyBlocked);
        return Result::DUTY_LIMIT;
    }
    if (!_r->send(buf, n)) {
        LOGE("tx_radio_err", "TX %s %u>%u seq=%u RADIO_ERR", ",\"type\":\"%s\",\"src\":%u,\"dst\":%u,\"seq\":%u", typeStr(f.type), f.src, f.dst, f.seq);
        return Result::RADIO_ERR;
    }
    _budgetUs -= at; _st.txFrames++;
    LOGD("tx", "TX %s %u>%u seq=%u ttl=%u flags=0x%02X lhs=0x%X lvl=%u plen=%u air=%uB/%luus budget=%ldus", ",\"type\":\"%s\",\"src\":%u,\"dst\":%u,\"seq\":%u,\"ttl\":%u,\"flags\":%u,\"lhs\":%u,\"lvl\":%u,\"plen\":%u,\"air_bytes\":%u,\"air_us\":%lu,\"budget_us\":%ld", typeStr(f.type), f.src, f.dst, f.seq, f.ttl, f.flags, f.lhs, level, f.plen, n, (unsigned long)at, (long)_budgetUs);
    return Result::OK;
}

Result Node::send(uint8_t dst, const uint8_t* payload, uint8_t len, bool reliable) {
    if (dst == BROADCAST || _cfg.mode == Mode::RAW) reliable = false;
    if (len > maxPayload(adaptive() ? 0 : LEVEL_NONE) || (len && !payload)) {
        LOGW("send_rejected", "SEND dst=%u len=%u rejected: TOO_LONG/null payload", ",\"dst\":%u,\"len\":%u", dst, len);
        return Result::TOO_LONG;
    }

    Frame f;
    f.src = _cfg.id; f.dst = dst; f.seq = _seq++; f.ttl = _cfg.defaultTtl;
    f.type = MsgType::DATA; f.flags = reliable ? FLAG_ACK_REQ : 0;
    f.plen = len; if (len) memcpy(f.payload, payload, len);
    seenOrAdd(f.src, f.seq);

    const uint8_t tries = reliable ? 1 + _cfg.maxRetries : 1;
    LOGI("send", "SEND dst=%u seq=%u len=%u reliable=%u tries=%u", ",\"dst\":%u,\"seq\":%u,\"len\":%u,\"reliable\":%u,\"tries\":%u", dst, f.seq, len, reliable, tries);

    for (uint8_t a = 0; a < tries; a++) {
        Peer& pr = peer(dst);
        f.lhs = pr.rxFb & 0x0F;
        if (a) { _st.txRetries++; LOGI("send_retry", "SEND seq=%u retry %u/%u", ",\"seq\":%u,\"retry\":%u,\"max_retries\":%u", f.seq, a, tries - 1); }

        if (reliable) { _wait = Wait(); _wait.active = true; _wait.dst = dst; _wait.seq = f.seq; }
        const uint8_t lvl = levelFor(pr);
        LOGD("send_attempt", "SEND seq=%u attempt %u/%u peerLvl=%u streak=%u lhs=0x%X", ",\"seq\":%u,\"attempt\":%u,\"tries\":%u,\"peer_lvl\":%u,\"streak\":%u,\"lhs\":%u", f.seq, a + 1, tries, lvl, pr.streak, f.lhs);
        Result r = transmit(f, lvl);
        if (r != Result::OK) {
            _wait.active = false;
            LOGW("send_aborted", "SEND seq=%u aborted: %s", ",\"seq\":%u,\"reason\":\"%s\"", f.seq, resultStr(r));
            return r;
        }
        if (!reliable) return Result::OK;

        uint32_t t0 = _p.millis();
        while (!_wait.ack && !_wait.nack && (uint32_t)(_p.millis() - t0) < _cfg.ackTimeoutMs) {
            processRx();
            if (_p.yield) _p.yield();
        }
        _wait.active = false;
        const uint32_t waited = (uint32_t)(_p.millis() - t0);
        Peer& after = peer(dst);
        if (_wait.ack) {
            LOGI("send_ok", "SEND seq=%u ACK from %u rtt=%lums fb=0x%X attempts=%u", ",\"seq\":%u,\"dst\":%u,\"rtt_ms\":%lu,\"fb\":%u,\"attempts\":%u", f.seq, dst, (unsigned long)waited, _wait.fb, a + 1);
            if (adaptive()) stepLevel(after.level, after.streak, _wait.fb, "tx", dst);
            return Result::OK;
        }
        if (_wait.nack) {
            LOGW("send_nack", "SEND seq=%u NACK after %lums fb=0x%X -> retransmit", ",\"seq\":%u,\"waited_ms\":%lu,\"fb\":%u", f.seq, (unsigned long)waited, _wait.fb);
            if (adaptive()) {
                const uint8_t before = after.level;
                after.streak = 0; nackJump(after.level);
                LOGI("lvl_nack_jump", "LVL NACK_JUMP peer=%u lvl %u->%u", ",\"peer\":%u,\"from\":%u,\"to\":%u", dst, before, after.level);
            }
            continue;                                                                    // retransmit at once
        }
        _st.timeouts++;
        LOGW("send_timeout", "SEND seq=%u TIMEOUT after %lums (timeouts=%lu)", ",\"seq\":%u,\"waited_ms\":%lu,\"timeouts\":%lu", f.seq, (unsigned long)waited, (unsigned long)_st.timeouts);
        if (adaptive()) {                                                                // "0x0": max protection
            LOGL(after.level != MAX_LEVEL ? 3 : 4, "lvl_timeout", "LVL TIMEOUT peer=%u lvl %u->%u",
                 ",\"peer\":%u,\"from\":%u,\"to\":%u", dst, after.level, (unsigned)MAX_LEVEL);
            after.streak = 0; after.level = MAX_LEVEL;
        }
    }
    LOGE("send_fail", "SEND seq=%u dst=%u NO_ACK after %u tries", ",\"seq\":%u,\"dst\":%u,\"tries\":%u", f.seq, dst, tries);
    return Result::NO_ACK;
}

// ------------------------------------------------------------------ RX path
void Node::poll() { processRx(); pumpRelay(); }

void Node::processRx() {
    uint8_t buf[AIR_MAX]; CC1101::RxInfo ri;
    for (uint8_t guard = 0; guard < 4; guard++) {
        int16_t n = _r->receive(buf, sizeof buf, &ri);
        if (n <= 0) return;
        handleAir(buf, (uint8_t)n, ri);
    }
}

void Node::handleAir(const uint8_t* buf, uint8_t n, const CC1101::RxInfo& ri) {
    Frame f; DecodeInfo di;
    switch (decodeFrame(buf, n, f, di)) {
        case DecodeStatus::OK: break;
        case DecodeStatus::TOO_SHORT:
        case DecodeStatus::BAD_HEADER:                                       // silent: sender times out
            _st.rxBadHeader++;
            LOGW("rx_bad_header", "RX bad header len=%u rssi=%d lqi=%u (total=%lu)", ",\"len\":%u,\"rssi\":%d,\"lqi\":%u,\"total\":%lu", n, ri.rssiDbm, (unsigned)ri.lqi, (unsigned long)_st.rxBadHeader);
            return;
        case DecodeStatus::ECC_FAIL: {
            _st.rxEccFail++;
            uint8_t fb = feedback(_lhs.score(0, di.t, ri.rssiDbm, ri.lqi, true), ri.rssiDbm);
            LOGW("rx_ecc_fail", "RX ECC_FAIL len=%u lvl=%u t=%u rssi=%d lqi=%u fb=0x%X (total=%lu)", ",\"len\":%u,\"lvl\":%u,\"ecc_t\":%u,\"rssi\":%d,\"lqi\":%u,\"fb\":%u,\"total\":%lu", n, (unsigned)di.level, (unsigned)di.t, ri.rssiDbm, (unsigned)ri.lqi, fb, (unsigned long)_st.rxEccFail);
            if (adaptive()) {
                if (_wait.active) LOGD("nack_skipped", "NACK skipped: waiting for ACK seq=%u (corrupted frame is most likely it)", ",\"wait_seq\":%u", _wait.seq);
                else              sendNack(fb);
            }
            return;
        }
        case DecodeStatus::CRC_FAIL: {
            _st.rxCrcFail++;
            uint8_t fb = feedback(_lhs.score(0, di.t, ri.rssiDbm, ri.lqi, true), ri.rssiDbm);
            LOGW("rx_crc_fail", "RX CRC_FAIL len=%u lvl=%u t=%u rssi=%d lqi=%u fb=0x%X (total=%lu)", ",\"len\":%u,\"lvl\":%u,\"ecc_t\":%u,\"rssi\":%d,\"lqi\":%u,\"fb\":%u,\"total\":%lu", n, (unsigned)di.level, (unsigned)di.t, ri.rssiDbm, (unsigned)ri.lqi, fb, (unsigned long)_st.rxCrcFail);
            if (adaptive()) {
                if (_wait.active) LOGD("nack_skipped", "NACK skipped: waiting for ACK seq=%u (corrupted frame is most likely it)", ",\"wait_seq\":%u", _wait.seq);
                else              sendNack(fb);
            }
            return;
        }
    }
    _st.rxOk++; _st.eccCorrected += di.corrected;
    uint8_t fb = feedback(_lhs.score(di.corrected, di.t, ri.rssiDbm, ri.lqi, false), ri.rssiDbm);
    if (f.type == MsgType::ACK && f.plen)
        LOGD("rx_ack", "RX ACK %u>%u ackFor=%u ttl=%u corr=%u lvl=%u t=%u rssi=%d lqi=%u fb=0x%X", ",\"src\":%u,\"dst\":%u,\"ack_for\":%u,\"ttl\":%u,\"corr\":%u,\"lvl\":%u,\"ecc_t\":%u,\"rssi\":%d,\"lqi\":%u,\"fb\":%u", f.src, f.dst, f.payload[0], f.ttl, (unsigned)di.corrected, (unsigned)di.level, (unsigned)di.t, ri.rssiDbm, (unsigned)ri.lqi, fb);
    else
        LOGD("rx", "RX %s %u>%u seq=%u ttl=%u plen=%u corr=%u lvl=%u t=%u rssi=%d lqi=%u fb=0x%X", ",\"type\":\"%s\",\"src\":%u,\"dst\":%u,\"seq\":%u,\"ttl\":%u,\"plen\":%u,\"corr\":%u,\"lvl\":%u,\"ecc_t\":%u,\"rssi\":%d,\"lqi\":%u,\"fb\":%u", typeStr(f.type), f.src, f.dst, f.seq, f.ttl, f.plen, (unsigned)di.corrected, (unsigned)di.level, (unsigned)di.t, ri.rssiDbm, (unsigned)ri.lqi, fb);

    if (f.type == MsgType::NACK) {                                            // link-local, never relayed
        if (_wait.active) {
            _wait.nack = true; _wait.fb = f.lhs; _st.nacksRx++;
            LOGD("rx_nack_used", "RX NACK from %u consumed by pending send seq=%u", ",\"src\":%u,\"wait_seq\":%u", f.src, _wait.seq);
        } else {
            LOGD("rx_nack_ignored", "RX NACK from %u ignored (no pending send)", ",\"src\":%u", f.src);
        }
        return;
    }
    if (f.src == _cfg.id) { LOGD("rx_own", "RX own frame seq=%u ignored", ",\"seq\":%u", f.seq); return; }
    Peer& from = peer(f.src);
    from.rxFb = fb;
    if (adaptive()) {
        stepLevel(_hopLevel, _hopStreak, fb, "hop", 0);                           // relay level
        stepLevel(from.ackLevel, from.ackStreak, fb, "ack", f.src);               // level of our replies to f.src
    }
    bool dup = seenOrAdd(f.src, f.seq);
    bool forMe = (f.dst == _cfg.id), bcast = (f.dst == BROADCAST);
    if (dup) LOGD("rx_dup", "RX dup %u seq=%u", ",\"src\":%u,\"seq\":%u", f.src, f.seq);

    if (f.type == MsgType::ACK && forMe) {
        if (_wait.active && f.plen >= 1 && f.payload[0] == _wait.seq && f.src == _wait.dst) {
            _wait.ack = true; _wait.fb = f.lhs; _st.acksRx++;
            LOGD("ack_match", "RX ACK from %u for seq=%u fb=0x%X", ",\"src\":%u,\"ack_for\":%u,\"fb\":%u", f.src, f.payload[0], f.lhs);
        } else {
            LOGD("ack_stale", "RX ACK from %u stale/unexpected (ackSeq=%u waitActive=%u waitSeq=%u)", ",\"src\":%u,\"ack_for\":%u,\"wait_active\":%u,\"wait_seq\":%u", f.src, f.plen ? f.payload[0] : 0, _wait.active, _wait.seq);
        }
        return;
    }
    if (f.type == MsgType::DATA && (forMe || bcast)) {
        LOGI("rx_data", "RX DATA from %u seq=%u plen=%u %s dup=%u fb=0x%X",
             ",\"src\":%u,\"seq\":%u,\"plen\":%u,\"cast\":\"%s\",\"dup\":%u,\"fb\":%u",
             f.src, f.seq, f.plen, bcast ? "bcast" : "unicast", (unsigned)dup, fb);
        if (forMe && (f.flags & FLAG_ACK_REQ)) sendAck(f, fb);                // re-ACK duplicates too
        if (dup) _st.rxDup++; else if (_cb) _cb(_cbCtx, f.src, f.payload, f.plen, fb);
        if (!forMe && !dup) queueRelay(f, fb);
        return;
    }
    if (!dup) queueRelay(f, fb);
}

void Node::sendAck(const Frame& rx, uint8_t fb) {
    Frame a;
    a.src = _cfg.id; a.dst = rx.src; a.seq = _seq++; a.ttl = _cfg.defaultTtl;
    a.type = MsgType::ACK; a.lhs = fb; a.plen = 1; a.payload[0] = rx.seq;
    seenOrAdd(a.src, a.seq);
    Peer& to = peer(rx.src);
    Result r = transmit(a, adaptive() ? to.ackLevel : LEVEL_NONE);                // was: to.level (never stepped on a pure receiver)
    if (r == Result::OK) { _st.acksTx++; LOGD("ack_sent", "ACK sent to %u for seq=%u fb=0x%X lvl=%u", ",\"dst\":%u,\"ack_for\":%u,\"fb\":%u,\"lvl\":%u", rx.src, rx.seq, fb, adaptive() ? to.ackLevel : (unsigned)LEVEL_NONE); }
    else                 { LOGW("ack_failed", "ACK to %u for seq=%u failed: %s", ",\"dst\":%u,\"ack_for\":%u,\"reason\":\"%s\"", rx.src, rx.seq, resultStr(r)); }
}

void Node::sendNack(uint8_t fb) {
    uint32_t now = _p.millis();
    if (_nackEver && (uint32_t)(now - _lastNack) < _cfg.nackHoldoffMs) {
        LOGD("nack_suppressed", "NACK suppressed (holdoff %lums left)", ",\"holdoff_left_ms\":%lu", (unsigned long)(_cfg.nackHoldoffMs - (uint32_t)(now - _lastNack)));
        return;
    }
    _lastNack = now; _nackEver = true;
    Frame a;
    a.src = _cfg.id; a.dst = BROADCAST; a.seq = _seq++; a.ttl = 0;
    a.type = MsgType::NACK; a.lhs = fb; a.plen = 0;
    Result r = transmit(a, MAX_LEVEL);
    if (r == Result::OK) { _st.nacksTx++; LOGI("nack_sent", "NACK sent fb=0x%X (total=%lu)", ",\"fb\":%u,\"total\":%lu", fb, (unsigned long)_st.nacksTx); }
    else                 { LOGW("nack_failed", "NACK send failed: %s", ",\"reason\":\"%s\"", resultStr(r)); }
}

// ------------------------------------------------------------------ relay (store & forward)
void Node::queueRelay(Frame f, uint8_t fb) {
    if (f.ttl == 0) { LOGD("relay_skip", "RELAY skip %u>%u seq=%u: ttl=0", ",\"src\":%u,\"dst\":%u,\"seq\":%u", f.src, f.dst, f.seq); return; }
    for (auto& s : _rq) if (!s.used) {
        f.ttl--; f.lhs = fb & 0x0F; s.f = f; s.used = true;
        const uint32_t jitter = _p.random((uint32_t)(_cfg.relayJitterMaxMs - _cfg.relayJitterMinMs + 1));
        const uint32_t delay  = _cfg.relayJitterMinMs + jitter;
        s.due = _p.millis() + delay;
        LOGD("relay_queued", "RELAY queued %s %u>%u seq=%u ttl=%u in %lums", ",\"type\":\"%s\",\"src\":%u,\"dst\":%u,\"seq\":%u,\"ttl\":%u,\"delay_ms\":%lu", typeStr(f.type), f.src, f.dst, f.seq, f.ttl, (unsigned long)delay);
        return;
    }
    _st.relayDropped++;
    LOGW("relay_full", "RELAY queue full: dropped %u>%u seq=%u (total=%lu)", ",\"src\":%u,\"dst\":%u,\"seq\":%u,\"total\":%lu", f.src, f.dst, f.seq, (unsigned long)_st.relayDropped);
}

void Node::pumpRelay() {
    for (auto& s : _rq) {
        if (!s.used || (int32_t)(_p.millis() - s.due) < 0) continue;
        Peer tmp; tmp.level = _hopLevel;
        Result r = transmit(s.f, levelFor(tmp));
        if (r == Result::OK) {
            _st.relayed++;
            LOGI("relay_fwd", "RELAY fwd %s %u>%u seq=%u ttl=%u lvl=%u", ",\"type\":\"%s\",\"src\":%u,\"dst\":%u,\"seq\":%u,\"ttl\":%u,\"lvl\":%u", typeStr(s.f.type), s.f.src, s.f.dst, s.f.seq, s.f.ttl, levelFor(tmp));
        } else {
            _st.relayDropped++;
            LOGW("relay_drop", "RELAY drop %u>%u seq=%u: %s", ",\"src\":%u,\"dst\":%u,\"seq\":%u,\"reason\":\"%s\"", s.f.src, s.f.dst, s.f.seq, resultStr(r));
        }
        s.used = false;
    }
}

} // namespace afec
