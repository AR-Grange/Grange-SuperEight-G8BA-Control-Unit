/**
 * @file    engine_protection.c
 * @brief   Module 6 — Engine Protection Implementation
 *
 * Protection evaluation order (highest priority first):
 *   1. Sync lost → immediate hard cut
 *   2. CLT overtemp → graduated cut
 *   3. Oil pressure low → graduated cut
 *   4. Oil temp high → power reduction
 *   5. Rev limit → soft/hard cut
 */

#include "engine_protection.h"
#include "fuel_injection.h"
#include "ignition_control.h"
#include "ignition_map.h"      /* IG-2 FIX: sole source of soft-cut cylinder masks */
#include "crank_cam_sync.h"
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * IG-2 FIX: SOFT_CUT_MASK_A/B removed from this module.
 *
 * Problem: engine_protection.c previously defined its own masks (0x55/0xAA)
 * AND ignition_map.c defined firing-order-aware masks (0x69/0x96).  Both
 * modules called ignition_set_soft_cut_mask() independently, causing the last
 * writer to win on each 10ms tick — producing an undefined mix of the two mask
 * patterns and breaking the even torque-reduction guarantee.
 *
 * Fix: rev_limiter_update() now calls ign_map_update_rev_limiter() for the
 * mask.  ignition_map.c is the SOLE owner of soft-cut mask state and toggle
 * logic.  engine_protection.c remains the owner of fuel-cut and overall
 * rev-limit mode decisions.
 */

/** Oil pressure: at low RPM (idle), lower pressure is normal — use this map */
static const uint16_t OIL_PRESS_RPM_AXIS[]  = {600, 1000, 2000, 3000, 7800};
static const float    OIL_PRESS_MIN_KPA[] = {80,  120,  160,  180,  200};
#define OIL_PRESS_POINTS  5u

/** Hysteresis for CLT (°C), oil press (kPa), oil temp (°C) — avoids rapid switching */
#define CLT_HYSTERESIS_C        3.0f
#define OIL_PRESS_HYSTERESIS    20.0f
#define RPM_HYSTERESIS          150u
/*
 * OT-1 FIX: Oil temperature hysteresis band.
 *
 * Without hysteresis, oil_temp_level transitions WARN↔OK every 50ms when
 * temperature oscillates ±1°C around G8BA_OIL_TEMP_WARN_C (130°C).
 * calc_power_reduction() adds 20% reduction at WARN level — the engine would
 * oscillate between 100% and 80% output power at 20 Hz, causing drivability
 * issues and excessive load on the power reduction logic.
 *
 * Fix: oil temp level only returns to OK when temperature falls below
 * (G8BA_OIL_TEMP_WARN_C − OIL_TEMP_HYSTERESIS_C) = 125°C.
 */
#define OIL_TEMP_HYSTERESIS_C   5.0f

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile prot_status_t g_protection;

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Get RPM-dependent minimum oil pressure */
static float oil_press_min_for_rpm(rpm_t rpm)
{
    for (uint8_t i = 0u; i < OIL_PRESS_POINTS - 1u; i++) {
        if (rpm < OIL_PRESS_RPM_AXIS[i+1u]) {
            float frac = ((float)rpm - (float)OIL_PRESS_RPM_AXIS[i])
                       / ((float)OIL_PRESS_RPM_AXIS[i+1u] - (float)OIL_PRESS_RPM_AXIS[i]);
            return OIL_PRESS_MIN_KPA[i] + frac *
                   (OIL_PRESS_MIN_KPA[i+1u] - OIL_PRESS_MIN_KPA[i]);
        }
    }
    return OIL_PRESS_MIN_KPA[OIL_PRESS_POINTS - 1u];
}

/** Evaluate CLT protection level */
static prot_level_t eval_clt(float clt_c, prot_level_t prev)
{
    if (clt_c >= G8BA_CLT_CUTOFF_C) {
        return PROT_LEVEL_CUT;
    } else if (clt_c >= G8BA_CLT_PROTECT_C) {
        return PROT_LEVEL_REDUCE;
    } else if (clt_c >= G8BA_CLT_WARN_C) {
        return PROT_LEVEL_WARN;
    } else {
        /* Apply hysteresis — don't drop level until well below threshold */
        if (prev == PROT_LEVEL_WARN &&
            clt_c < (G8BA_CLT_WARN_C - CLT_HYSTERESIS_C)) {
            return PROT_LEVEL_OK;
        } else if (prev > PROT_LEVEL_OK) {
            return (clt_c < (G8BA_CLT_WARN_C - CLT_HYSTERESIS_C))
                   ? PROT_LEVEL_OK : prev;
        }
        return PROT_LEVEL_OK;
    }
}

/** Evaluate oil pressure protection level */
static prot_level_t eval_oil_press(float press_kpa, rpm_t rpm, prot_level_t prev)
{
    /* Don't check oil pressure below cranking speed */
    if (rpm < G8BA_RPM_CRANK) return PROT_LEVEL_OK;

    float min_press = oil_press_min_for_rpm(rpm);

    /*
     * C-4 FIX: original code was `(min_press - G8BA_OIL_PRESS_MIN_KPA)` which
     * produced a cut threshold of only 50 kPa at 7800 RPM (200 - 150 = 50).
     * The RPM-dependent table already encodes the minimum safe pressure at each
     * speed; we must cut when actual pressure falls below that value directly.
     */
    if (press_kpa < min_press) {
        return PROT_LEVEL_CUT;
    } else if (press_kpa < (G8BA_OIL_PRESS_WARN_KPA)) {
        return PROT_LEVEL_WARN;
    } else {
        if (prev == PROT_LEVEL_WARN &&
            press_kpa > (G8BA_OIL_PRESS_WARN_KPA + OIL_PRESS_HYSTERESIS)) {
            return PROT_LEVEL_OK;
        }
        return (prev > PROT_LEVEL_OK) ? prev : PROT_LEVEL_OK;
    }
}

/** Calculate power reduction percentage from protection levels */
static float calc_power_reduction(const prot_status_t *p)
{
    /* CLT-based reduction */
    float pwr = 0.0f;

    if (p->thermal.clt_level == PROT_LEVEL_REDUCE) {
        /* Linear reduction: 0% at warn threshold → 50% at cutoff */
        float clt = p->inputs.clt_c;
        float frac = (clt - G8BA_CLT_WARN_C) / (G8BA_CLT_CUTOFF_C - G8BA_CLT_WARN_C);
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        pwr = frac * 50.0f;
    }

    /* Oil temp additional reduction */
    if (p->thermal.oil_temp_level >= PROT_LEVEL_WARN) {
        pwr += 20.0f;
    }

    if (pwr > 100.0f) pwr = 100.0f;
    return pwr;
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t engine_protection_init(void)
{
    memset((void *)&g_protection, 0, sizeof(g_protection));
    g_protection.overall_level = PROT_LEVEL_OK;
    g_protection.rev_limiter.hysteresis_rpm = G8BA_RPM_SOFT_CUT - RPM_HYSTERESIS;
    return G8BA_OK;
}

void engine_protection_update(const prot_inputs_t *inputs)
{
    if (inputs == NULL) return;

    g_protection.inputs = *inputs;
    prot_event_flags_t events = PROT_EVENT_NONE;

    /* ── 1. Sync lost check ─────────────────────────────────────────────── */
    if (!inputs->sync_ok) {
        prot_emergency_cut();
        events |= PROT_EVENT_SYNC_LOST;
        g_protection.active_events = events;
        g_protection.overall_level = PROT_LEVEL_EMERG;
        return;
    } else {
        prot_emergency_clear();
        fuel_cut_clear(FCUT_SYNC_LOST);
    }

    /* ── 2. CLT evaluation ──────────────────────────────────────────────── */
    prot_level_t clt_lv = eval_clt(inputs->clt_c, g_protection.thermal.clt_level);
    g_protection.thermal.clt_level = clt_lv;

    switch (clt_lv) {
    case PROT_LEVEL_CUT:
        events |= (PROT_EVENT_CLT_WARN | PROT_EVENT_CLT_REDUCE | PROT_EVENT_CLT_CUT);
        fuel_cut_set(FCUT_OVERTEMP);
        g_protection.clt_warn_count++;
        prot_log_fault(PROT_EVENT_CLT_CUT, PROT_LEVEL_CUT, inputs->clt_c);
        break;
    case PROT_LEVEL_REDUCE:
        events |= (PROT_EVENT_CLT_WARN | PROT_EVENT_CLT_REDUCE);
        fuel_cut_clear(FCUT_OVERTEMP);
        break;
    case PROT_LEVEL_WARN:
        events |= PROT_EVENT_CLT_WARN;
        fuel_cut_clear(FCUT_OVERTEMP);
        g_protection.clt_warn_count++;
        prot_send_warning(PROT_EVENT_CLT_WARN);
        break;
    default:
        fuel_cut_clear(FCUT_OVERTEMP);
        break;
    }

    /* ── 3. Oil pressure evaluation ─────────────────────────────────────── */
    prot_level_t oil_lv = eval_oil_press(inputs->oil_pressure_kpa, inputs->rpm,
                                         g_protection.thermal.oil_press_level);
    g_protection.thermal.oil_press_level = oil_lv;

    switch (oil_lv) {
    case PROT_LEVEL_CUT:
        events |= (PROT_EVENT_OIL_PRESS_W | PROT_EVENT_OIL_PRESS_CUT);
        fuel_cut_set(FCUT_LOW_OIL);
        g_protection.oil_warn_count++;
        prot_log_fault(PROT_EVENT_OIL_PRESS_CUT, PROT_LEVEL_CUT,
                       inputs->oil_pressure_kpa);
        break;
    case PROT_LEVEL_WARN:
        events |= PROT_EVENT_OIL_PRESS_W;
        fuel_cut_clear(FCUT_LOW_OIL);
        g_protection.oil_warn_count++;
        prot_send_warning(PROT_EVENT_OIL_PRESS_W);
        break;
    default:
        fuel_cut_clear(FCUT_LOW_OIL);
        break;
    }

    /* ── 4. Oil temperature ─────────────────────────────────────────────── */
    /*
     * OT-1 FIX: retain previous oil_temp_level for hysteresis calculation.
     *
     * The level must only fall from WARN to OK when temperature drops below
     * (G8BA_OIL_TEMP_WARN_C − OIL_TEMP_HYSTERESIS_C).  This prevents
     * rapid oscillation between 100% and 80% power output when oil temp
     * hovers near the 130°C warning threshold.
     */
    prot_level_t oil_temp_prev = g_protection.thermal.oil_temp_level;

    if (inputs->oil_temp_c >= G8BA_OIL_TEMP_MAX_C) {
        events |= (PROT_EVENT_OIL_TEMP_W | PROT_EVENT_OIL_TEMP_CUT);
        g_protection.thermal.oil_temp_level = PROT_LEVEL_REDUCE;
    } else if (inputs->oil_temp_c >= G8BA_OIL_TEMP_WARN_C) {
        events |= PROT_EVENT_OIL_TEMP_W;
        g_protection.thermal.oil_temp_level = PROT_LEVEL_WARN;
        prot_send_warning(PROT_EVENT_OIL_TEMP_W);
    } else {
        /* OT-1 FIX: apply hysteresis — only clear when below (WARN - hyst) */
        if (oil_temp_prev == PROT_LEVEL_OK ||
            inputs->oil_temp_c < (G8BA_OIL_TEMP_WARN_C - OIL_TEMP_HYSTERESIS_C)) {
            g_protection.thermal.oil_temp_level = PROT_LEVEL_OK;
        }
        /* else: remain at previous level until temp drops through hysteresis band */
    }

    /* ── 5. Power reduction calculation ─────────────────────────────────── */
    g_protection.thermal.power_reduction_pct = calc_power_reduction(
        (const prot_status_t *)&g_protection);

    /* ── 6. Overall level ────────────────────────────────────────────────── */
    prot_level_t worst = PROT_LEVEL_OK;
    if (clt_lv > worst)         worst = clt_lv;
    if (oil_lv > worst)         worst = oil_lv;
    if (g_protection.thermal.oil_temp_level > worst)
        worst = g_protection.thermal.oil_temp_level;

    g_protection.active_events  = events;
    g_protection.overall_level  = worst;
    g_protection.last_update_us = 0; /* replace: getTimeNowUs() */
}

void rev_limiter_update(rpm_t rpm)
{
    rev_limiter_state_t *rl = (rev_limiter_state_t *)&g_protection.rev_limiter;
    prot_event_flags_t *ev  = (prot_event_flags_t *)&g_protection.active_events;

    /*
     * IG-2 FIX: Delegate cylinder cut-mask computation to ignition_map module.
     *
     * ign_map_update_rev_limiter() is now the single point of truth for:
     *   - Soft-cut mask alternation (firing-order-aware 0x69/0x96)
     *   - Hard-cut mask (0x00)
     *   - Hysteresis state maintenance
     *   - Clear mask (0xFF) when below threshold
     *
     * This function retains ownership of:
     *   - Fuel cut flag management (FCUT_REV_LIMIT, FCUT_SOFT_CUT)
     *   - Ignition mode selection (IGN_MODE_SOFT_CUT / HARD_CUT / RUNNING)
     *   - Protection event flag updates
     *   - rev_limiter_state_t diagnostic fields
     */
    uint8_t mask = ign_map_update_rev_limiter(rpm);
    rl->soft_cut_mask = mask;

    if (rpm >= G8BA_RPM_MAX) {
        /* Hard cut */
        rl->hard_active = true;
        rl->soft_active = true;
        *ev |= PROT_EVENT_REV_HARD;
        fuel_cut_set(FCUT_REV_LIMIT);
        ignition_set_mode(IGN_MODE_HARD_CUT);
        /* mask = 0x00 from ign_map — not applied; IGN_MODE_HARD_CUT disables all */

    } else if (rpm >= G8BA_RPM_SOFT_CUT) {
        /* Soft cut — firing-order-aware alternating mask from ignition_map */
        rl->hard_active = false;
        rl->soft_active = true;
        rl->soft_cut_toggle ^= 1u;   /* keep diagnostic toggle in sync */
        *ev |= PROT_EVENT_REV_SOFT;
        fuel_cut_clear(FCUT_REV_LIMIT);
        fuel_cut_set(FCUT_SOFT_CUT);
        ignition_set_soft_cut_mask(mask);
        ignition_set_mode(IGN_MODE_SOFT_CUT);

    } else if (rpm < rl->hysteresis_rpm) {
        /* Clear rev limit */
        rl->hard_active = false;
        rl->soft_active = false;
        *ev &= ~(PROT_EVENT_REV_SOFT | PROT_EVENT_REV_HARD);
        fuel_cut_clear(FCUT_REV_LIMIT);
        fuel_cut_clear(FCUT_SOFT_CUT);
        ignition_set_soft_cut_mask(IGNMAP_ALL_CYL_MASK);
        ignition_set_mode(IGN_MODE_RUNNING);
    }
    /* else: inside hysteresis band — ign_map maintains its state, no action */
}

bool prot_is_hard_cut_active(void)
{
    return g_protection.rev_limiter.hard_active ||
           (g_protection.thermal.clt_level   == PROT_LEVEL_CUT) ||
           (g_protection.thermal.oil_press_level == PROT_LEVEL_CUT) ||
           (g_protection.active_events & PROT_EVENT_SYNC_LOST);
}

float prot_get_power_reduction(void)
{
    return g_protection.thermal.power_reduction_pct / 100.0f;
}

prot_level_t prot_get_level(void)
{
    return g_protection.overall_level;
}

prot_event_flags_t prot_get_events(void)
{
    return g_protection.active_events;
}

void prot_emergency_cut(void)
{
    fuel_cut_set(FCUT_SYNC_LOST);
    ignition_set_mode(IGN_MODE_HARD_CUT);
    g_protection.active_events |= PROT_EVENT_SYNC_LOST;
    g_protection.overall_level  = PROT_LEVEL_EMERG;
}

void prot_emergency_clear(void)
{
    /*
     * H-6 FIX: Do NOT call ignition_set_mode(IGN_MODE_RUNNING) here.
     *
     * Problem: this function is invoked at the top of engine_protection_update()
     * as soon as sync is re-established, BEFORE the CLT and oil pressure checks
     * that follow. If any of those checks would trigger a cut/reduce state, we
     * would re-enable ignition for the duration of the function body — up to
     * one protection-update tick (50 ms) — even though the protective condition
     * is still present.
     *
     * Correct responsibility boundary:
     *   prot_emergency_clear() — only clears the SYNC_LOST flag and resets the
     *                            emergency level so subsequent checks can
     *                            compute the real overall level.
     *   thd_medium_ctrl (main.c) — sole owner of the engine state machine;
     *                            calls ignition_set_mode() based on the
     *                            final overall_level after ALL evaluations.
     */
    if (crank_get_sync_state() == SYNC_FULL) {
        g_protection.active_events &= ~PROT_EVENT_SYNC_LOST;
        if (g_protection.overall_level == PROT_LEVEL_EMERG) {
            g_protection.overall_level = PROT_LEVEL_OK;
        }
        /* ignition mode restored by thd_medium_ctrl state machine, not here */
    }
}

void prot_send_warning(prot_event_flags_t event)
{
    /*
     * Drive warning light output and/or send CAN frame.
     * efiSetPinValue(WARNING_LED_PIN, true);
     * can_send_protection_warning(event);
     */
    (void)event;
}

void prot_log_fault(prot_event_flags_t event, prot_level_t level, float value)
{
    /*
     * Write to non-volatile fault log (e.g. EEPROM or flash page).
     * In RusEFI, use the stored fault codes mechanism:
     * engine->faultCodes.add(event);
     */
    (void)event; (void)level; (void)value;
}
