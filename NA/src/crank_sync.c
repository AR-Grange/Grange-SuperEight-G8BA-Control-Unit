/**
 * @file    crank_sync.c
 * @brief   Hyundai Tau G8BA — 36-2 Crank / Cam Synchronisation Implementation
 *
 * ══════════════════════════════════════════════════════════════════════════
 * State machine overview
 * ══════════════════════════════════════════════════════════════════════════
 *
 *   NONE ──(gap confirmed ×2)──► HALF ──(cam phase confirmed ×2)──► FULL
 *    ▲                              │                                  │
 *    └──────────── sync_loss ───────┴──────────────────────────────────┘
 *
 * NONE  : Scanning for the 36-2 gap.  No angle or RPM output.
 *         We require 2 consecutive detected gaps (CRANK_SYNC_CONFIRM_GAPS)
 *         before advancing, so a single noise spike cannot false-lock.
 *
 * HALF  : Gap found and confirmed.  tooth_index 0-33 and crank revolution
 *         angle (0-359 °) are valid.  RPM is valid.  We cannot assign
 *         cylinder events because we don't know which of the two crank
 *         revolutions per 4-stroke cycle we are in.
 *
 * FULL  : A cam edge has been observed at a consistent crank angle in the
 *         same revolution (×2 confirmations).  Full 0-719 ° engine-cycle
 *         angle is valid.  All cylinder events may be scheduled.
 *
 * ══════════════════════════════════════════════════════════════════════════
 * Gap detection algorithm
 * ══════════════════════════════════════════════════════════════════════════
 *
 *   For 36-2:  gap spans 3 theoretical tooth positions (10° each = 30°).
 *              gap_period  ≈  3 × normal_period
 *              gap_ratio   =  gap_period / prev_period  ≈  3.0
 *
 *   Accept as gap:   CRANK_GAP_RATIO_MIN (1.8) ≤ ratio ≤ CRANK_GAP_RATIO_MAX (4.5)
 *   Reject as noise: ratio > 4.5  (engine stalled or wiring fault)
 *
 *   After accepting a gap event the CURRENT period ended with tooth_index 0
 *   (the first physical tooth that arrived after the two missing positions).
 *   All subsequent periods are normal teeth (tooth_index 1…33).
 *
 * ══════════════════════════════════════════════════════════════════════════
 * RPM calculation
 * ══════════════════════════════════════════════════════════════════════════
 *
 *   From a single tooth period T_us (excluding gap-crossing periods):
 *
 *     RPM = 60,000,000 / (T_us × CRANK_TEETH_TOTAL)
 *         = 60,000,000 / (T_us × 36)
 *
 *   A rolling average over RPM_WINDOW_TEETH (8) teeth smooths noise while
 *   remaining responsive.  The oldest entry is replaced each tooth.
 *
 * ══════════════════════════════════════════════════════════════════════════
 * Angle interpolation
 * ══════════════════════════════════════════════════════════════════════════
 *
 *   Between tooth events the angle advances linearly at the speed implied
 *   by the most recent tooth period:
 *
 *     angle_360 = (tooth_index × 10.0)
 *               + ((now_us - last_tooth_us) / last_period_us) × 10.0
 *
 *   This gives sub-degree resolution between ISR events.
 *
 * ══════════════════════════════════════════════════════════════════════════
 * Cam phase logic
 * ══════════════════════════════════════════════════════════════════════════
 *
 *   The intake cam makes one revolution per 720 ° of crank.  Each bank's
 *   intake cam fires a Hall edge once per engine cycle at approximately:
 *
 *     Bank A (B1-IN):  CAM_A_REF_ANGLE_DEG ± CAM_PHASE_TOLERANCE_DEG
 *     Bank B (B2-IN):  CAM_B_REF_ANGLE_DEG ± CAM_PHASE_TOLERANCE_DEG
 *
 *   The reference angles below assume 0° VVT advance (parked/home).
 *   At full advance the edge may shift by up to 50°; the ±60° tolerance
 *   window accommodates the full D-CVVT advance range.
 *
 *   When a cam edge arrives:
 *     1. Record which crank revolution (0 or 1) it fell in.
 *     2. Compare with the previous observation for the same bank.
 *     3. If they agree twice in a row → set engine_phase and promote to FULL.
 *
 *   If crank angle at cam edge < 360° → engine_phase = FIRST  (revolution 0)
 *   If crank angle at cam edge ≥ 360° → engine_phase = SECOND (revolution 1)
 *   … but since cam_cb gets angle_360 (not angle_720), we use the revolution
 *   counter s_revolution_count (odd/even) to determine the phase half.
 */

#include "crank_sync.h"
#include <string.h>
#include <math.h>

/*
 * CS-C6 FIX: extern declaration moved to file scope so it is visible to the
 * compiler at the point of use and appears in dependency analysis.
 * Previously this was buried inside crank_sync_get_angle_360(), hiding the
 * dependency from static analysis tools and violating MISRA-C Rule 8.4.
 *
 * RusEFI: replace with #include "os_util.h" which exposes getTimeNowUs().
 */
extern uint32_t getTimeNowUs(void);

/* ══════════════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * CS-C16 FIX: g8ba_config.h independently defines G8BA_TRIGGER_TEETH_TOTAL,
 * G8BA_TRIGGER_TEETH_PHYSICAL, and G8BA_TRIGGER_TOOTH_ANGLE for the same
 * wheel.  If someone updates one file and forgets the other the decoder
 * will silently use the wrong geometry.
 *
 * These compile-time checks catch any divergence immediately.
 */
#if (CRANK_TEETH_TOTAL    != G8BA_TRIGGER_TEETH_TOTAL)   || \
    (CRANK_TEETH_MISSING  != G8BA_TRIGGER_TEETH_MISSING) || \
    (CRANK_TEETH_PHYSICAL != G8BA_TRIGGER_TEETH_PHYSICAL)
#  error "Wheel constant mismatch: crank_sync.h and g8ba_config.h disagree — fix both files together"
#endif

/** Cam edge reference angles (crank angle_360, °) at 0° VVT advance.
 *  MUST be calibrated against the actual engine before production.
 *  These values are plausible starting points for the G8BA V8 layout.   */
#define CAM_A_REF_ANGLE_DEG   90.0f   /* Bank 1 intake cam ref (crank °)  */
#define CAM_B_REF_ANGLE_DEG  270.0f   /* Bank 2 intake cam ref (crank °)  */

/** RPM rolling-average window (number of teeth, must be power of 2) */
#define RPM_WINDOW_TEETH    8u
#define RPM_WINDOW_MASK     (RPM_WINDOW_TEETH - 1u)

/** Minimum plausible tooth period — below this is noise (> ~20 000 RPM) */
#define MIN_TOOTH_PERIOD_US  100u

/** Maximum plausible tooth period — above this = stall (< ~30 RPM) */
#define MAX_TOOTH_PERIOD_US  2000000u   /* 2 seconds */

/* ══════════════════════════════════════════════════════════════════════════
 * CYLINDER TDC TABLE
 * ══════════════════════════════════════════════════════════════════════════
 *
 * Firing order 1-2-7-8-4-5-6-3  →  zero-based: {0,1,6,7,3,4,5,2}
 * Firing offsets from cyl-1 TDC: 0, 90, 180, 270, 360, 450, 540, 630 °
 *
 * Absolute 720° angle = G8BA_TDC_CYL1_OFFSET_DEG + firing_offset (mod 720)
 *
 *   cyl 0 (1):  114 + 0   = 114 °
 *   cyl 1 (2):  114 + 90  = 204 °
 *   cyl 2 (3):  114 + 630 = 744 → 24 °  (wraps)
 *   cyl 3 (4):  114 + 360 = 474 °
 *   cyl 4 (5):  114 + 450 = 564 °
 *   cyl 5 (6):  114 + 540 = 654 °
 *   cyl 6 (7):  114 + 180 = 294 °
 *   cyl 7 (8):  114 + 270 = 384 °
 */
const float CRANK_CYL_TDC_DEG[G8BA_CYLINDERS] = {
    114.0f,   /* cyl 1  (index 0) — firing position 1                */
    204.0f,   /* cyl 2  (index 1) — firing position 2                */
     24.0f,   /* cyl 3  (index 2) — firing position 8 (744 mod 720)  */
    474.0f,   /* cyl 4  (index 3) — firing position 5                */
    564.0f,   /* cyl 5  (index 4) — firing position 6                */
    654.0f,   /* cyl 6  (index 5) — firing position 7                */
    294.0f,   /* cyl 7  (index 6) — firing position 3                */
    384.0f,   /* cyl 8  (index 7) — firing position 4                */
};

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile crank_sync_status_t g_crank_sync;

/* ── Tooth tracking ──────────────────────────────────────────────────────── */
static uint32_t  s_last_tooth_us;         /* timestamp of last tooth edge    */
static uint32_t  s_prev_period_us;        /* tooth period before last edge   */
static uint8_t   s_tooth_index;           /* 0-33 within current revolution  */
static uint32_t  s_revolution_count;      /* revolutions since sync locked   */
/*
 * CS-C4 FIX: removed `s_next_is_tooth_zero` — it was declared and reset
 * but never written or read anywhere in the implementation (dead code).
 * Its role is fully covered by the gap-detection flow in crank_sync_tooth_cb.
 */

/* ── Gap search ──────────────────────────────────────────────────────────── */
static uint8_t   s_gap_confirm;           /* confirmed consecutive gaps      */
static bool      s_sync_locked;           /* tooth 0 position known          */

/* ── RPM filter ──────────────────────────────────────────────────────────── */
static uint32_t  s_period_buf[RPM_WINDOW_TEETH]; /* circular buffer, µs     */
static uint8_t   s_period_head;           /* next write index                */
static uint32_t  s_period_sum;            /* running sum of buffer contents  */
static uint8_t   s_period_fill;           /* how many entries are valid      */

/* ── Cam phase ───────────────────────────────────────────────────────────── */
static engine_phase_t  s_cam_phase_vote[CAM_BANK_COUNT];   /* last vote      */
static uint8_t         s_cam_confirm[CAM_BANK_COUNT];       /* confirm count  */

/*
 * CS-C17 FIX: track which revolution each cam bank last fired.
 * Compared against s_revolution_count in handle_gap() to detect
 * cam sensors that have gone silent (wire break, sensor fault).
 * Fault declared if no edge seen for > CAM_FAULT_REVOLUTIONS crank revs.
 */
#define CAM_FAULT_REVOLUTIONS   4u   /* 2 engine cycles without cam edge = fault */
static uint32_t  s_cam_last_rev[CAM_BANK_COUNT]; /* revolution at last edge  */

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Add one tooth period to the RPM rolling-average window */
static void rpm_window_add(uint32_t period_us)
{
    /* Remove oldest entry from sum */
    s_period_sum -= s_period_buf[s_period_head];

    /* Insert new entry */
    s_period_buf[s_period_head] = period_us;
    s_period_sum                += period_us;

    s_period_head = (s_period_head + 1u) & RPM_WINDOW_MASK;
    if (s_period_fill < RPM_WINDOW_TEETH) s_period_fill++;
}

/** Compute RPM from rolling-average tooth period */
static rpm_t rpm_from_window(void)
{
    if (s_period_fill == 0u) return 0u;

    uint32_t avg_us = s_period_sum / s_period_fill;
    if (avg_us == 0u) return 0u;

    /*
     * RPM = 60,000,000 / (avg_period_us × CRANK_TEETH_TOTAL)
     * 36 theoretical teeth per revolution × 60 s/min × 1,000,000 µs/s
     * = 60,000,000 / (T × 36)
     */
    return (rpm_t)(60000000UL / ((uint64_t)avg_us * CRANK_TEETH_TOTAL));
}

/**
 * Commit a new tooth_index and update the public angle fields.
 * Called with interrupts already locked (inside crank_sync_tooth_cb).
 */
static void commit_tooth(uint8_t idx, uint32_t period_us)
{
    s_tooth_index = idx;
    g_crank_sync.tooth_index       = idx;
    g_crank_sync.tooth_period_us   = period_us;

    /* angle_360 is exact at the tooth edge (interpolation occurs in getter) */
    float a360 = (float)idx * CRANK_DEG_PER_TOOTH;
    g_crank_sync.angle_360 = a360;

    /* angle_720 depends on engine phase */
    float a720;
    if (g_crank_sync.engine_phase == ENGINE_PHASE_SECOND) {
        a720 = a360 + 360.0f;
    } else {
        a720 = a360;  /* ENGINE_PHASE_FIRST or UNKNOWN — treat as first */
    }
    g_crank_sync.angle_720 = a720;
}

/**
 * Handle gap detection — called when the current period is ≥ 1.8 × previous.
 * The tooth that TRIGGERED this period is tooth_index = 0.
 */
static void handle_gap(uint32_t timestamp_us, uint32_t period_us)
{
    (void)timestamp_us;

    bool was_locked = s_sync_locked;

    if (!s_sync_locked) {
        s_gap_confirm++;
        if (s_gap_confirm >= CRANK_SYNC_CONFIRM_GAPS) {
            s_sync_locked = true;
            g_crank_sync.sync_state = CRANK_SYNC_HALF;

            /*
             * CS-C12 FIX: reset s_revolution_count to 0 at the moment sync
             * first locks, giving it a known starting parity.
             *
             * Before this fix, s_revolution_count could be any value N at
             * the moment of lock (one increment per detected gap, including
             * the pre-confirmation search phase).  The cam phase logic uses
             * parity (even→FIRST, odd→SECOND) to determine which crank
             * revolution the cam edge fell in.  An arbitrary starting parity
             * would make every-other engine cycle the wrong assignment,
             * causing systematic cylinder misidentification.
             *
             * After this fix: the confirming gap is revolution 0 (even),
             * so the first cam edge after sync will consistently resolve to
             * the correct phase once it has been observed twice.
             */
            s_revolution_count = 0u;
        }
    } else {
        /* Already synced — count this revolution */
        s_revolution_count++;
    }

    /*
     * CS-C17 FIX: check for cam sensor silence.
     * After the very first sync lock (was_locked false → true), skip this
     * check because s_cam_last_rev[] is still 0 (pre-sync baseline).
     */
    if (was_locked) {
        for (uint8_t b = 0u; b < CAM_BANK_COUNT; b++) {
            uint32_t revs_since = s_revolution_count - s_cam_last_rev[b];
            if (revs_since > CAM_FAULT_REVOLUTIONS) {
                g_crank_sync.cam[b].fault = true;
                /* Do NOT drop to HALF sync here — the crank angle is still
                 * valid.  The fault flag allows the caller to decide whether
                 * to freeze VVT or raise a warning.  A separate recovery
                 * path via crank_sync_reset() is the caller's responsibility. */
            }
        }
    }

    g_crank_sync.gap_just_seen = true;

    /*
     * CS-C13 FIX: the gap period spans 3 tooth positions (30°), so it is
     * approximately 3× the normal tooth period.  Storing it directly in
     * tooth_period_us (which the angle interpolation getter divides into
     * CRANK_DEG_PER_TOOTH = 10°) would make the interpolated angle advance
     * only 1/3 as fast as reality for the first tooth after each gap.
     *
     * Fix: store the estimated normal period (gap_period / 3).  The division
     * by the literal 3 matches the gap geometry: 2 missing + 1 tooth position.
     * This estimate is replaced by a true measured period on the very next
     * (non-gap) tooth, so any error lasts at most one 10° tooth interval.
     */
    uint32_t est_normal_period = period_us / 3u;
    if (est_normal_period == 0u) est_normal_period = 1u;   /* paranoia guard */
    commit_tooth(0u, est_normal_period);

    /* Gap period itself is not added to the RPM filter (not a normal tooth) */
}

/**
 * Handle a normal (non-gap) tooth in the locked state.
 */
static void handle_normal_tooth(uint32_t timestamp_us, uint32_t period_us)
{
    (void)timestamp_us;

    /* Advance index; wrap at CRANK_TEETH_PHYSICAL means we missed the gap */
    uint8_t next_idx = s_tooth_index + 1u;
    if (next_idx >= CRANK_TEETH_PHYSICAL) {
        /*
         * We expected the gap at tooth_index 33→34 but it never came.
         * Either the wheel has an extra tooth, the gap was too small to
         * detect, or an IRQ was missed.  Force a sync loss.
         */
        crank_sync_reset();
        g_crank_sync.sync_loss_count++;
        return;
    }

    commit_tooth(next_idx, period_us);
    rpm_window_add(period_us);

    /* Update public RPM */
    g_crank_sync.rpm            = rpm_from_window();
    g_crank_sync.gap_just_seen  = false;
}

/**
 * Evaluate a cam edge for engine-phase determination.
 * Called while ISR-locked (inside crank_sync_cam_cb).
 *
 * Phase vote logic:
 *   - The cam fires once per engine cycle.  We observe which crank revolution
 *     (odd or even, tracked by s_revolution_count) the edge falls in.
 *   - Even revolution → ENGINE_PHASE_FIRST
 *   - Odd  revolution → ENGINE_PHASE_SECOND
 *   Two consecutive agreeing votes promote to FULL sync.
 */
static void process_cam_edge(cam_bank_t bank)
{
    /* Only intake cams are needed for phase — reject out-of-range bank IDs */
    if (bank >= CAM_BANK_COUNT) return;

    /* Need at least HALF sync to interpret a cam edge */
    if (g_crank_sync.sync_state < CRANK_SYNC_HALF) return;

    /*
     * CS-C5 / CS-C10 FIX: Angle-window gate for cam edges.
     *
     * Previously CAM_A_REF_ANGLE_DEG and CAM_B_REF_ANGLE_DEG were defined
     * but never used — every rising cam edge (whenever it arrived in the
     * crank cycle) was treated as a valid phase vote.  A single EMI-induced
     * glitch at any crank angle would corrupt the phase assignment, causing
     * the engine to fire every cylinder 360° out of position.
     *
     * Fix: accept only cam edges that arrive within ±CAM_PHASE_TOLERANCE_DEG
     * of the expected crank angle for each bank.  With the D-CVVT system able
     * to advance up to 50°, CAM_PHASE_TOLERANCE_DEG = 60° comfortably covers
     * the full advance range while still rejecting noise that arrives in the
     * other 240° of the revolution.
     *
     * Note: CAM_A/B_REF_ANGLE_DEG must be calibrated on the actual engine.
     * The values 90°/270° are plausible starting points for the G8BA layout.
     */
    float ref_angle = (bank == CAM_BANK_A) ? CAM_A_REF_ANGLE_DEG
                                           : CAM_B_REF_ANGLE_DEG;
    float angle_now = g_crank_sync.angle_360;
    float deviation = angle_now - ref_angle;

    /* Fold into [-180, +180] range (single step sufficient for window < 180°) */
    if      (deviation >  180.0f) deviation -= 360.0f;
    else if (deviation < -180.0f) deviation += 360.0f;

    if (deviation < 0.0f) deviation = -deviation;   /* fabsf without math.h */

    if (deviation > CAM_PHASE_TOLERANCE_DEG) {
        /* Edge outside expected window — discard as noise / VVT overshoot */
        return;
    }

    /* Record revolution number for cam fault monitoring (CS-C17) */
    s_cam_last_rev[bank] = s_revolution_count;
    g_crank_sync.cam[bank].fault = false;

    /* Determine which revolution half this cam edge falls in */
    engine_phase_t vote = ((s_revolution_count & 1u) == 0u)
                          ? ENGINE_PHASE_FIRST
                          : ENGINE_PHASE_SECOND;

    /* Confirm debounce: two consecutive identical votes */
    if (vote == s_cam_phase_vote[bank]) {
        s_cam_confirm[bank]++;
    } else {
        /* Phase changed — start fresh confirmation */
        s_cam_phase_vote[bank] = vote;
        s_cam_confirm[bank]    = 1u;
    }

    g_crank_sync.cam[bank].phase_vote     = vote;
    g_crank_sync.cam[bank].confirm_count  = s_cam_confirm[bank];
    g_crank_sync.cam[bank].last_crank_angle = g_crank_sync.angle_360;

    if (s_cam_confirm[bank] >= CAM_CONFIRM_COUNT) {
        /* Phase confirmed by this bank's cam */
        g_crank_sync.engine_phase    = vote;
        g_crank_sync.phase_confirmed = true;
        g_crank_sync.sync_state      = CRANK_SYNC_FULL;

        /*
         * Recompute angle_720 immediately with the newly confirmed phase so
         * any caller that reads right after this call gets the correct value.
         */
        float a720 = g_crank_sync.angle_360
                   + ((vote == ENGINE_PHASE_SECOND) ? 360.0f : 0.0f);
        g_crank_sync.angle_720 = a720;

        s_cam_confirm[bank] = CAM_CONFIRM_COUNT; /* saturate, don't overflow */
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t crank_sync_init(void)
{
    crank_sync_reset();
    return G8BA_OK;
}

void crank_sync_reset(void)
{
    /*
     * CS-C8 FIX: concurrent-access documentation and protection.
     *
     * This function is called from two contexts:
     *   (A) ISR context  — via handle_normal_tooth() and crank_sync_tooth_cb()
     *                       when an overrun or implausible period is detected.
     *   (B) Thread context — external callers (e.g. stall detection in main.c).
     *
     * In context (A) the crank ISR is already the highest-priority active
     * code; no other ISR or thread can interleave.
     *
     * In context (B) the crank ISR CAN preempt and run handle_gap() or
     * handle_normal_tooth() concurrently with this function's multi-word
     * writes to s_* and g_crank_sync.  The memset + field restore sequence
     * is NOT atomic from the ISR's perspective.
     *
     * Required caller discipline for thread context:
     *   chSysLock();
     *   crank_sync_reset();
     *   chSysUnlock();
     *
     * This cannot be done inside the function itself because ChibiOS
     * chSysLock() is not re-entrant and is illegal from ISR context (A).
     * The caller must choose the appropriate path.  This contract is
     * documented in crank_sync.h.
     */

    /* Zero public status — preserves sync_loss_count as diagnostic */
    uint8_t saved_loss = g_crank_sync.sync_loss_count;
    memset((void *)&g_crank_sync, 0, sizeof(g_crank_sync));
    g_crank_sync.sync_loss_count = saved_loss;

    /* Zero private state */
    s_last_tooth_us     = 0u;
    s_prev_period_us    = 0u;
    s_tooth_index       = 0u;
    s_revolution_count  = 0u;
    s_gap_confirm       = 0u;
    s_sync_locked       = false;

    memset(s_period_buf,   0, sizeof(s_period_buf));
    memset(s_cam_last_rev, 0, sizeof(s_cam_last_rev));
    s_period_head = 0u;
    s_period_sum  = 0u;
    s_period_fill = 0u;

    for (uint8_t b = 0u; b < CAM_BANK_COUNT; b++) {
        s_cam_phase_vote[b] = ENGINE_PHASE_UNKNOWN;
        s_cam_confirm[b]    = 0u;
        g_crank_sync.cam[b].bank = (cam_bank_t)b;
    }

    g_crank_sync.sync_state        = CRANK_SYNC_NONE;
    g_crank_sync.engine_phase      = ENGINE_PHASE_UNKNOWN;
    g_crank_sync.gap_confirm_count = 0u;
}

/* ─────────────────────────────────────────────────────────────────────────
 * crank_sync_tooth_cb
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Called on every rising edge from the MAX9926 output (one call per
 * physical tooth, including the tooth that ends the gap).
 *
 * Execution context: ISR  (TIM input-capture or EXTI interrupt)
 * Must NOT block or acquire normal mutexes.
 *
 * CS-C1 FIX: the previous version claimed "Uses chSysLockFromISR" but did
 * not actually call it.  The correct concurrency model for this module is:
 *
 *   ISR writes:   Direct volatile writes to g_crank_sync and s_* variables.
 *                 On Cortex-M7 (STM32H743), aligned 32-bit stores are
 *                 single-instruction and cannot be torn by another ISR at
 *                 the same or lower priority.  Set the crank EXTI/TIM
 *                 priority HIGH enough that no other ISR preempts it.
 *
 *   Thread reads: Getters read individual volatile fields; each read is
 *                 atomic.  Multi-field consistency (angle + period + time)
 *                 is ensured by the guard `if (elapsed > tp) elapsed = tp`
 *                 in crank_sync_get_angle_360().
 *
 *   Thread reset: Callers of crank_sync_reset() from thread context MUST
 *                 wrap the call in chSysLock() / chSysUnlock() (see CS-C8).
 */
void crank_sync_tooth_cb(uint32_t timestamp_us)
{
    /* ── Compute inter-tooth period ─────────────────────────────────────── */
    uint32_t period_us = timestamp_us - s_last_tooth_us;
    s_last_tooth_us    = timestamp_us;

    /* Reject implausible periods (noise or stall) */
    if (period_us < MIN_TOOTH_PERIOD_US || period_us > MAX_TOOTH_PERIOD_US) {
        if (s_sync_locked) {
            crank_sync_reset();
            g_crank_sync.sync_loss_count++;
        }
        s_prev_period_us = 0u;
        return;
    }

    /* ── Gap detection ──────────────────────────────────────────────────── */
    if (s_prev_period_us > 0u) {
        float ratio = (float)period_us / (float)s_prev_period_us;

        if (ratio >= CRANK_GAP_RATIO_MIN && ratio <= CRANK_GAP_RATIO_MAX) {
            /*
             * GAP DETECTED.
             * The long period just ended with tooth_index = 0.
             * handle_gap() sets tooth_index = 0 and increments revolution.
             */
            handle_gap(timestamp_us, period_us);
            s_prev_period_us = 0u; /* gap period is not a valid tooth period */
            g_crank_sync.gap_confirm_count = s_gap_confirm;
            return;

        } else if (ratio > CRANK_STALL_RATIO) {
            /* Abnormally long — stall or disconnected sensor */
            if (s_sync_locked) {
                crank_sync_reset();
                g_crank_sync.sync_loss_count++;
            }
            s_prev_period_us = period_us;
            return;
        }
    }

    s_prev_period_us = period_us;

    /* ── Normal tooth processing ────────────────────────────────────────── */
    if (s_sync_locked) {
        handle_normal_tooth(timestamp_us, period_us);
    } else {
        /*
         * Not yet synced.  We're accumulating tooth periods to prime the
         * gap detector (need at least one valid prev_period_us before the
         * gap can be detected).  No angle output yet.
         */
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 * crank_sync_cam_cb
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Called on each Hall-sensor edge from a cam sensor.
 * Only the INTAKE cams (Bank A = B1-IN, Bank B = B2-IN) contribute to
 * engine-cycle phase determination; exhaust cams use the same callback
 * but are ignored if the bank ID maps to exhaust (caller should pass
 * only CAM_BANK_A or CAM_BANK_B for intake cams).
 *
 * Execution context: ISR  (TIM input-capture)
 */
void crank_sync_cam_cb(cam_bank_t bank, bool rising_edge,
                        uint32_t timestamp_us)
{
    if (bank >= CAM_BANK_COUNT) return;

    g_crank_sync.cam[bank].last_edge_us = timestamp_us;

    /*
     * Use the RISING edge of the cam pulse (tooth entering field).
     * Falling edge is ignored for phase detection; it may be used for
     * VVT duty-cycle measurement in a future extension.
     */
    if (!rising_edge) return;

    g_crank_sync.cam[bank].fault = false;

    process_cam_edge(bank);
}

/* ─────────────────────────────────────────────────────────────────────────
 * Getters
 * ─────────────────────────────────────────────────────────────────────────
 *
 * These are called from task context (threads), not from ISR.
 * The volatile qualifiers on g_crank_sync ensure reads are not cached
 * across the ISR boundary without needing a mutex (single-word reads
 * on Cortex-M are atomic; float reads are atomic on STM32H7 with FPU).
 */

float crank_sync_get_angle_360(void)
{
    if (g_crank_sync.sync_state < CRANK_SYNC_HALF) return 0.0f;

    /*
     * Interpolate between the last tooth edge and now.
     *
     * elapsed_us / tooth_period_us  gives the fraction of one tooth spacing
     * that has elapsed since the last edge.
     * Multiply by CRANK_DEG_PER_TOOTH to convert to degrees.
     */
    uint32_t tp = g_crank_sync.tooth_period_us;
    if (tp == 0u) return g_crank_sync.angle_360;

    /* Read timestamp — getTimeNowUs() declared at file scope (CS-C6 fix) */
    uint32_t now_us  = getTimeNowUs();
    uint32_t elapsed = now_us - s_last_tooth_us;

    /* Guard against elapsed > tp (missed tooth ISR or timer wrap) */
    if (elapsed > tp) elapsed = tp;

    float frac = (float)elapsed / (float)tp;
    float interp = g_crank_sync.angle_360 + frac * CRANK_DEG_PER_TOOTH;

    if (interp >= 360.0f) interp -= 360.0f;
    return interp;
}

float crank_sync_get_angle_720(void)
{
    if (g_crank_sync.sync_state < CRANK_SYNC_FULL) return 0.0f;

    float a360 = crank_sync_get_angle_360();
    float a720 = (g_crank_sync.engine_phase == ENGINE_PHASE_SECOND)
               ? a360 + 360.0f
               : a360;

    if (a720 >= 720.0f) a720 -= 720.0f;
    return a720;
}

float crank_sync_get_tdc_deg(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return 0.0f;
    return CRANK_CYL_TDC_DEG[cyl_index];
}

float crank_sync_deg_to_tdc(uint8_t cyl_index)
{
    if (!crank_sync_is_full_sync() || cyl_index >= G8BA_CYLINDERS) {
        return 720.0f;
    }

    float tdc   = CRANK_CYL_TDC_DEG[cyl_index];
    float now   = crank_sync_get_angle_720();
    float delta = tdc - now;

    /* Normalise to (0, 720] — always positive (degrees to NEXT TDC) */
    if (delta <= 0.0f) delta += 720.0f;
    return delta;
}

rpm_t crank_sync_get_rpm(void)
{
    return g_crank_sync.rpm;
}

crank_sync_state_t crank_sync_get_state(void)
{
    return g_crank_sync.sync_state;
}

bool crank_sync_is_full_sync(void)
{
    return g_crank_sync.sync_state == CRANK_SYNC_FULL;
}

engine_phase_t crank_sync_get_phase(void)
{
    return g_crank_sync.engine_phase;
}
