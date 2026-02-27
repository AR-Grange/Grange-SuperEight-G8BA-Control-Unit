/**
 * @file    crank_sync.h
 * @brief   Hyundai Tau G8BA — 36-2 Crank / Cam Synchronisation
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Hardware signal chain
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   Crank VR sensor                      STM32H743 TIM1 CH1
 *   (magnetic pick-up)  ──►  MAX9926  ──►  (IC capture, rising edge)
 *                         VR conditioner   GPIO → EXTI (fallback)
 *
 *   Cam Hall sensors  ──►  3.3 V pull-up  ──►  TIM2/TIM3/TIM4/TIM5 CH1
 *   (B1-IN, B1-EX,        Schmitt trigger      (IC capture, both edges)
 *    B2-IN, B2-EX)
 *
 * MAX9926 notes:
 *   - Adaptive zero-crossing threshold — no external bias resistor needed
 *   - VROUT is a push-pull digital output at 3.3 V logic
 *   - Rising edge on VROUT corresponds to the LEADING edge of each tooth
 *     entering the sensor field (i.e. leading-edge firing convention)
 *   - SEN pin: pull to GND for 2.0 V threshold, leave open for auto-adapt
 *
 * ──────────────────────────────────────────────────────────────────────────
 * 36-2 wheel geometry (crankshaft-mounted, 1:1 with crank)
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   Theoretical positions : 36  (10 °/position)
 *   Missing teeth         : 2   (adjacent, at positions 34 and 35)
 *   Physical teeth        : 34
 *   Gap angular span      : 30 ° (positions 34, 35, and to tooth 0 = 3 T)
 *
 *   One wheel revolution = 360 °  →  tooth period T = 360/(36 × RPM/60) s
 *   Gap period ≈ 3 T  →  gap ratio threshold : 1.8 … 4.5
 *
 *   Tooth indexing (this module):
 *     tooth_index 0   = first tooth after the gap (reference position)
 *     tooth_index 1…33 = subsequent physical teeth
 *
 *   Crank angle within a revolution:
 *     angle_360 = tooth_index × CRANK_DEG_PER_TOOTH + fractional (interpolated)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * 720 ° engine-cycle tracking
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   The crank makes 2 revolutions per 4-stroke cycle (720 °).
 *   Both crank revolutions produce identical 36-2 patterns; the cam sensor
 *   is required to distinguish them (engine_phase_t).
 *
 *   angle_720 = tooth_index × 10.0 + (engine_phase × 360.0)
 *               + fractional interpolation
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Cylinder TDC table  (firing order 1-2-7-8-4-5-6-3)
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   Each cylinder's compression-TDC angle is:
 *     CRANK_CYL_TDC_DEG[cyl] = G8BA_TDC_CYL1_OFFSET_DEG + firing_offset[cyl]
 *
 *   firing_offset[]   = {0, 90, 630, 360, 450, 540, 180, 270} ° for
 *                        cyl {1,  2,   3,   4,   5,   6,   7,   8}
 *   (index 0 = cyl 1, index 7 = cyl 8)
 *
 *   With G8BA_TDC_CYL1_OFFSET_DEG = 114 °:
 *     cyl1(0)=114, cyl2(1)=204, cyl3(2)=24 (744 mod 720),
 *     cyl4(3)=474, cyl5(4)=564, cyl6(5)=654, cyl7(6)=294, cyl8(7)=384
 *
 * ──────────────────────────────────────────────────────────────────────────
 * RusEFI integration
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   1. Register crank_sync_tooth_cb with TriggerCentral's shaft-signal handler
 *   2. Register crank_sync_cam_cb for each cam input
 *   3. Timestamps: RusEFI's efitick_t  →  uint32_t µs via NT2US() before call
 *   4. Call crank_sync_init() from initEngineController()
 */

#ifndef CRANK_SYNC_H
#define CRANK_SYNC_H

#include "g8ba_config.h"   /* rpm_t, floatdeg_t, g8ba_status_t, G8BA_CYLINDERS */

/* ══════════════════════════════════════════════════════════════════════════
 * WHEEL GEOMETRY CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

#define CRANK_TEETH_TOTAL       36u        /**< Theoretical positions / rev    */
#define CRANK_TEETH_MISSING     2u         /**< Missing adjacent teeth         */
#define CRANK_TEETH_PHYSICAL    34u        /**< Actual rising edges / rev      */
#define CRANK_DEG_PER_TOOTH     10.0f      /**< Degrees per theoretical tooth  */

/**
 * Gap detection thresholds.
 * Gap spans 3 normal tooth periods (2 missing + next tooth).
 * Ratio = gap_period / prev_normal_period ≈ 3.0
 */
#define CRANK_GAP_RATIO_MIN     1.8f       /**< Below = normal tooth           */
#define CRANK_GAP_RATIO_MAX     4.5f       /**< Above = noise / stall          */

/** Number of confirmed gaps required before declaring HALF sync */
#define CRANK_SYNC_CONFIRM_GAPS 2u

/** Max permitted tooth period jump (× factor) before counting as noise/stall */
#define CRANK_STALL_RATIO       8.0f

/** CAM edge angle tolerance (± degrees) for phase window matching */
#define CAM_PHASE_TOLERANCE_DEG 60.0f

/** Number of consecutive agreeing cam edges required for FULL sync */
#define CAM_CONFIRM_COUNT       2u

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

/** Crank / cam synchronisation state */
typedef enum {
    CRANK_SYNC_NONE = 0,   /**< Searching — angle and RPM unknown            */
    CRANK_SYNC_HALF,       /**< 36-2 gap locked; crank 0-359° known, RPM OK,
                                but engine-cycle phase (720°) unknown         */
    CRANK_SYNC_FULL,       /**< Full 0-719° engine-cycle angle known          */
} crank_sync_state_t;

/** Engine-cycle half (which of the 2 crank revolutions per 4-stroke cycle) */
typedef enum {
    ENGINE_PHASE_UNKNOWN = 0,
    ENGINE_PHASE_FIRST,    /**< First revolution  (angle_720  =   0°…359°)   */
    ENGINE_PHASE_SECOND,   /**< Second revolution (angle_720  = 360°…719°)   */
} engine_phase_t;

/** Cam bank identifiers */
typedef enum {
    CAM_BANK_A = 0,        /**< Bank 1 — cylinders 1-4 (intake cam used)     */
    CAM_BANK_B = 1,        /**< Bank 2 — cylinders 5-8 (intake cam used)     */
    CAM_BANK_COUNT = 2,
} cam_bank_t;

/** Per-cam-bank tracking */
typedef struct {
    cam_bank_t      bank;
    uint32_t        last_edge_us;        /**< Timestamp of last cam edge      */
    float           last_crank_angle;    /**< Crank angle_360 at that edge    */
    engine_phase_t  phase_vote;          /**< Phase this cam edge suggested   */
    uint8_t         confirm_count;       /**< Consecutive agreeing votes      */
    bool            fault;              /**< No edge seen in > 720 °          */
} cam_sync_state_t;

/** Full module status (read-only for callers) */
typedef struct {
    /* Synchronisation */
    crank_sync_state_t  sync_state;
    engine_phase_t      engine_phase;

    /* Position */
    float       angle_360;           /**< 0-359.9 °, crank revolution angle  */
    float       angle_720;           /**< 0-719.9 °, full engine-cycle angle  */
    uint8_t     tooth_index;         /**< 0-33, within current revolution     */

    /* Speed */
    rpm_t       rpm;                  /**< Filtered engine speed (RPM)        */
    uint32_t    tooth_period_us;      /**< Last tooth-to-tooth period (µs)    */

    /* Diagnostics */
    bool        gap_just_seen;        /**< Set for one tooth after gap event  */
    uint8_t     sync_loss_count;      /**< Total sync-loss events             */
    uint8_t     gap_confirm_count;    /**< Confirmed gap events (0-2)         */
    bool        phase_confirmed;      /**< Cam phase was confirmed            */

    /* Per-bank cam state */
    cam_sync_state_t  cam[CAM_BANK_COUNT];
} crank_sync_status_t;

/* ══════════════════════════════════════════════════════════════════════════
 * CYLINDER TDC TABLE
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * Compression-TDC angle in the 720 ° engine cycle for each cylinder.
 *
 * Index:  0=Cyl1, 1=Cyl2, 2=Cyl3, 3=Cyl4, 4=Cyl5, 5=Cyl6, 6=Cyl7, 7=Cyl8
 * Firing order encoded:  1-2-7-8-4-5-6-3
 * Reference: tooth_index 0 = 0° in this table's coordinate system.
 *
 * @note   G8BA_TDC_CYL1_OFFSET_DEG (114 °) must be verified against the
 *         actual engine with a dial gauge and timing light before production.
 */
extern const float CRANK_CYL_TDC_DEG[G8BA_CYLINDERS];

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

extern volatile crank_sync_status_t g_crank_sync;

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise crank/cam synchronisation module.
 *         Clears all state; safe to call at power-on and after stall.
 * @return G8BA_OK always (future: G8BA_ERR_HW if timer init fails).
 */
g8ba_status_t crank_sync_init(void);

/**
 * @brief  Reset to CRANK_SYNC_NONE (e.g. on stall or sync loss).
 *         Does not reconfigure hardware.
 *
 * @warning CONCURRENCY CONTRACT (CS-C8):
 *   This function is called internally from ISR context (crank_sync_tooth_cb).
 *   If you call it from thread context you MUST wrap it in a critical section:
 *
 *     chSysLock();
 *     crank_sync_reset();
 *     chSysUnlock();
 *
 *   Failure to do so allows the crank ISR to preempt the memset mid-execution,
 *   producing a partially-cleared state and incorrect angle/RPM output.
 */
void crank_sync_reset(void);

/**
 * @brief  Crank tooth callback — call on every rising edge from MAX9926.
 *
 *         RusEFI integration:
 *           In the TriggerCentral handler (C++ side):
 *             extern "C" void crank_sync_tooth_cb(uint32_t ts_us);
 *             crank_sync_tooth_cb((uint32_t)NT2US(timestamp));
 *
 *         ISR safety: this function manipulates module state under a
 *         critical section.  Do NOT call from a task context.
 *
 * @param  timestamp_us   Hardware timestamp of the tooth edge (µs, wraps OK).
 */
void crank_sync_tooth_cb(uint32_t timestamp_us);

/**
 * @brief  Cam edge callback — call on each cam Hall-sensor edge.
 *
 *         Only the INTAKE cams (Bank A = B1-IN, Bank B = B2-IN) are needed
 *         for engine-cycle phase determination.  Exhaust cam edges are
 *         ignored if cam_bank < CAM_BANK_COUNT.
 *
 * @param  bank           CAM_BANK_A or CAM_BANK_B
 * @param  rising_edge    true = rising (cam tooth entering sensor field)
 * @param  timestamp_us   Hardware timestamp (µs, same timebase as tooth_cb)
 */
void crank_sync_cam_cb(cam_bank_t bank, bool rising_edge,
                        uint32_t timestamp_us);

/* ── Angle / position ────────────────────────────────────────────────────── */

/**
 * @brief  Return current crank angle in 0-359.9 ° (revolution angle).
 *         Interpolated between tooth events for sub-degree resolution.
 *         Returns 0 if sync state < CRANK_SYNC_HALF.
 */
float crank_sync_get_angle_360(void);

/**
 * @brief  Return current engine-cycle angle in 0-719.9 °.
 *         Returns 0 if sync state < CRANK_SYNC_FULL.
 */
float crank_sync_get_angle_720(void);

/**
 * @brief  Return compression-TDC angle (0-719 °) for the given cylinder.
 * @param  cyl_index   0-based cylinder index (0=Cyl1 … 7=Cyl8)
 * @return TDC angle, or 0.0f if cyl_index is out of range.
 */
float crank_sync_get_tdc_deg(uint8_t cyl_index);

/**
 * @brief  Degrees to next compression-TDC for the given cylinder.
 *         Accounts for 720 ° wrap.  Returns 720.0f if not FULL sync.
 * @param  cyl_index   0-based cylinder index
 */
float crank_sync_deg_to_tdc(uint8_t cyl_index);

/* ── RPM ─────────────────────────────────────────────────────────────────── */

/**
 * @brief  Return filtered engine speed (RPM).
 *         Uses rolling average of last 8 tooth periods.
 *         Returns 0 if sync state is NONE.
 */
rpm_t crank_sync_get_rpm(void);

/* ── State ───────────────────────────────────────────────────────────────── */

/** Return current synchronisation state. */
crank_sync_state_t crank_sync_get_state(void);

/** Return true if engine-cycle angle is fully known (FULL sync). */
bool crank_sync_is_full_sync(void);

/** Return current engine-cycle phase (which crank revolution). */
engine_phase_t crank_sync_get_phase(void);

#endif /* CRANK_SYNC_H */
