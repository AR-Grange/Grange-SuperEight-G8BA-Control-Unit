/**
 * @file    g8ba_config.h
 * @brief   Hyundai Tau 4.6 G8BA V8 — Master Engine Configuration
 *
 * Hardware: RusEFI Proteus V0.4+
 * RTOS:     ChibiOS/RT
 * Target:   STM32H743 (Proteus)
 *
 * Firing order : 1-2-7-8-4-5-6-3
 * Trigger wheel: 36-2 VR on crankshaft
 * Cam sensors  : Hall effect × 4 (B1 IN, B1 EX, B2 IN, B2 EX)
 */

#ifndef G8BA_CONFIG_H
#define G8BA_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ── RusEFI / ChibiOS includes ─────────────────────────────────────────── */
#include "ch.h"           /* ChibiOS RTOS core          */
#include "hal.h"          /* ChibiOS HAL                */
#include "efi_gpio.h"     /* RusEFI GPIO abstraction    */
#include "sensor.h"       /* RusEFI sensor API          */
#include "engine.h"       /* RusEFI engine object       */

/* ══════════════════════════════════════════════════════════════════════════
 * ENGINE MECHANICAL CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_CYLINDERS          8
#define G8BA_DISPLACEMENT_CC    4627u          /* cm³                        */
#define G8BA_BORE_MM            92.0f          /* mm                         */
#define G8BA_STROKE_MM          87.0f          /* mm                         */
#define G8BA_COMPRESSION_RATIO  10.4f
#define G8BA_CRANK_ANGLE_CYCLE  720.0f         /* degrees per full cycle     */
#define G8BA_FIRING_INTERVAL    90.0f          /* °CA between events (720/8) */

/**
 * Firing order encoded as 0-based cylinder indices.
 * Physical cylinders: 1-2-7-8-4-5-6-3 → indices: 0,1,6,7,3,4,5,2
 * Bank 1: cyl 1,2,3,4  (indices 0-3)
 * Bank 2: cyl 5,6,7,8  (indices 4-7)
 */
#define G8BA_FIRING_ORDER       {0u, 1u, 6u, 7u, 3u, 4u, 5u, 2u}
#define G8BA_FIRING_ORDER_STR   "1-2-7-8-4-5-6-3"

/* ══════════════════════════════════════════════════════════════════════════
 * TRIGGER WHEEL — 36-2 VR CRANK
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_TRIGGER_TEETH_TOTAL    36u        /* nominal tooth count        */
#define G8BA_TRIGGER_TEETH_MISSING  2u         /* missing teeth for sync     */
#define G8BA_TRIGGER_TEETH_PHYSICAL 34u        /* actual teeth present       */
#define G8BA_TRIGGER_TOOTH_ANGLE    (360.0f / G8BA_TRIGGER_TEETH_TOTAL)  /* 10° */
#define G8BA_TRIGGER_GAP_ANGLE      (G8BA_TRIGGER_TOOTH_ANGLE * (G8BA_TRIGGER_TEETH_MISSING + 1u))

/** Crank angle of cylinder #1 TDC compression stroke (calibrate on dyno) */
#define G8BA_TDC_CYL1_OFFSET_DEG   114.0f

/* ══════════════════════════════════════════════════════════════════════════
 * REV LIMITS
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_RPM_MAX                7800u      /* hard rev limit (rpm)       */
#define G8BA_RPM_SOFT_CUT           7600u      /* soft cut start (rpm)       */
#define G8BA_RPM_IDLE_TARGET        800u       /* warm idle target           */
#define G8BA_RPM_CRANK              400u       /* cranking threshold (rpm)   */

/* ══════════════════════════════════════════════════════════════════════════
 * FUEL SYSTEM
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_INJECTOR_FLOW_CC_MIN   650.0f     /* cc/min @ 300 kPa — Siemens/Continental high-flow MPI (FC-1 FIX: was 440 stock) */
#define G8BA_FUEL_PRESSURE_KPA      300.0f     /* rail pressure (kPa gauge)  */
#define G8BA_STOICH_AFR             14.7f      /* lambda 1.0 for gasoline    */
#define G8BA_INJ_DEAD_TIME_US       600u       /* injector dead time (µs)    */
#define G8BA_INJ_MIN_PW_US          800u       /* minimum pulse width (µs)   */
#define G8BA_INJ_MAX_PW_US          25000u     /* maximum pulse width (µs)   */

/* ══════════════════════════════════════════════════════════════════════════
 * IGNITION SYSTEM
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_IGN_COIL_DWELL_MS      3.5f       /* coil dwell time (ms)       */
#define G8BA_IGN_ADVANCE_MAX        45.0f      /* max advance (°BTDC)        */
#define G8BA_IGN_ADVANCE_CRANK      5.0f       /* cranking advance (°BTDC)   */
#define G8BA_IGN_ADVANCE_IDLE       12.0f      /* idle advance (°BTDC)       */

/* ══════════════════════════════════════════════════════════════════════════
 * D-CVVT PARAMETERS
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_CVVT_BANKS             2u         /* Bank1, Bank2               */
#define G8BA_CVVT_CAMS_PER_BANK     2u         /* intake + exhaust           */
#define G8BA_CVVT_TOTAL_ACTUATORS   4u         /* 4 VVT solenoids            */
#define G8BA_CVVT_PWM_HZ            250u       /* solenoid PWM frequency     */
#define G8BA_CVVT_IN_ADVANCE_MAX    50.0f      /* intake max advance (°CA)   */
#define G8BA_CVVT_EX_RETARD_MAX     30.0f      /* exhaust max retard (°CA)   */
#define G8BA_CVVT_DEADBAND_DEG      1.5f       /* position deadband (°CA)    */
/* Racing PID gains — high KP for fast transient response, low KI to avoid
 * integrator lag, moderate KD to damp the aggressive proportional action.
 * Update rate: TASK_PERIOD_FAST_MS = 5 ms.  Deadband = 1.5 °CA.
 * Overshoot is acceptable; minimising settling time is the priority. */
#define G8BA_CVVT_KP                3.5f       /* PID proportional gain      */
#define G8BA_CVVT_KI                0.08f      /* PID integral gain          */
#define G8BA_CVVT_KD                0.30f      /* PID derivative gain        */

/* ══════════════════════════════════════════════════════════════════════════
 * KNOCK CONTROL
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_KNOCK_SENSORS          2u         /* one per bank               */
#define G8BA_KNOCK_FREQ_HZ          6800u      /* characteristic frequency   */
#define G8BA_KNOCK_RETARD_STEP      1.5f       /* degrees per knock event    */
#define G8BA_KNOCK_ADVANCE_STEP     0.3f       /* recovery step (°/cycle)    */
#define G8BA_KNOCK_RETARD_MAX       10.0f      /* max total retard (°CA)     */
#define G8BA_KNOCK_WINDOW_ATDC      10.0f      /* window start (°ATDC)       */
#define G8BA_KNOCK_WINDOW_END       60.0f      /* window end (°ATDC)         */

/* ══════════════════════════════════════════════════════════════════════════
 * ENGINE PROTECTION THRESHOLDS
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_CLT_WARN_C             105.0f     /* coolant warning (°C)       */
#define G8BA_CLT_PROTECT_C          112.0f     /* power reduction (°C)       */
#define G8BA_CLT_CUTOFF_C           118.0f     /* fuel cut (°C)              */
#define G8BA_OIL_PRESS_MIN_KPA      150.0f     /* min oil pressure (kPa)     */
#define G8BA_OIL_PRESS_WARN_KPA     200.0f     /* oil press warning (kPa)    */
#define G8BA_OIL_TEMP_WARN_C        130.0f     /* oil temp warning (°C)      */
#define G8BA_OIL_TEMP_MAX_C         145.0f     /* oil temp cutoff (°C)       */
#define G8BA_BOOST_MAX_KPA          250.0f     /* MAP limit (kPa abs)        */

/* ══════════════════════════════════════════════════════════════════════════
 * TASK SCHEDULING (ChibiOS thread periods)
 * ══════════════════════════════════════════════════════════════════════════ */

#define TASK_PERIOD_FAST_MS         5u         /* 200 Hz — knock/CVVT ctrl   */
#define TASK_PERIOD_MEDIUM_MS       10u        /* 100 Hz — fuel/ign update   */
#define TASK_PERIOD_SLOW_MS         50u        /* 20 Hz  — protection/diag   */
#define TASK_PERIOD_VERYLOW_MS      250u       /* 4 Hz   — logging/telemetry */

/* Thread stack sizes — 1 kB minimum for threads with FPU + function calls */
#define STACK_CRANK_SYNC            1024u
#define STACK_FUEL_INJ              1024u
#define STACK_IGNITION              1024u
#define STACK_CVVT                  1024u
#define STACK_KNOCK                 1024u
#define STACK_PROTECTION            768u

/* ══════════════════════════════════════════════════════════════════════════
 * FUEL CALCULATION PHYSICAL CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Air density at 20°C, 101.325 kPa (g/cc) — reference for base PW */
#define AIR_DENSITY_REF_G_CC        0.001204f
/** Gasoline density (g/cc) — used to convert fuel mass → injector volume */
#define FUEL_DENSITY_G_CC           0.720f
/** Reference MAP pressure for base PW normalisation */
#define MAP_REF_KPA                 100.0f

/* ══════════════════════════════════════════════════════════════════════════
 * CAM PHASE CONFIRMATION PARAMETERS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Cam edge must arrive within ±this many °CA of expected angle to be valid */
#define CAM_PHASE_CONFIRM_WINDOW_DEG  30.0f
/** Number of consecutive valid confirmations before phase is locked */
#define CAM_PHASE_MIN_CONFIRMS        2u

/* ══════════════════════════════════════════════════════════════════════════
 * COMMON TYPE ALIASES
 * ══════════════════════════════════════════════════════════════════════════ */

typedef float   floatdeg_t;    /* crank angle degrees          */
typedef float   floatms_t;     /* time in milliseconds         */
typedef float   floatv_t;      /* voltage                      */
typedef uint32_t rpm_t;        /* engine speed rpm             */
typedef uint32_t us_t;         /* time in microseconds         */

/** Standardized return codes for all G8BA modules */
typedef enum {
    G8BA_OK           = 0,
    G8BA_ERR_SENSOR   = -1,
    G8BA_ERR_RANGE    = -2,
    G8BA_ERR_TIMEOUT  = -3,
    G8BA_ERR_SYNC     = -4,
    G8BA_ERR_OVERTEMP = -5,
    G8BA_ERR_LOWPRESS = -6,
    G8BA_ERR_HW       = -7,   /* hardware peripheral did not respond to init */
} g8ba_status_t;

/*
 * HW-1 FIX: phaser_id_t promoted from dcvvt_control.h to this global header.
 * Both the control layer (dcvvt_control.h) and the hardware layer (dcvvt_hw.h)
 * require this type.  Defining it here breaks the layering inversion that
 * previously forced dcvvt_hw.h to include dcvvt_control.h.
 */
/** D-CVVT oil-control-valve channel identifiers (match cam_id_t 0-based) */
typedef enum {
    PHASER_B1_INTAKE  = 0,   /**< Bank1 intake cam phaser  */
    PHASER_B1_EXHAUST = 1,   /**< Bank1 exhaust cam phaser */
    PHASER_B2_INTAKE  = 2,   /**< Bank2 intake cam phaser  */
    PHASER_B2_EXHAUST = 3,   /**< Bank2 exhaust cam phaser */
    PHASER_COUNT      = 4,   /**< Total number of phasers  */
} phaser_id_t;

#endif /* G8BA_CONFIG_H */
