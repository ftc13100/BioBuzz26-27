//package org.firstinspires.ftc.teamcode.autotune;
//
//import com.aaravdhawan25.pidautotuner.ftc.dashboard.AutoTuneDash;
//import com.aaravdhawan25.pidautotuner.ftc.dashboard.AutoTuneDashboard;
//import com.acmerobotics.dashboard.config.Config;
//
///**
// * Edit these values for your robot before running any tuner OpMode.
// *
// * <p>The {@code @Config} annotation makes every {@code public static}
// * field live-tunable via FTC Dashboard (http://192.168.43.1:8080/dash)
// * while the OpMode is running -- no redeploy needed.
// *
// * <p>This is the ONLY file you need to copy into your TeamCode.
// * Everything else (PIDMaster, DualMotorPIDMaster, the algorithms) comes
// * from the {@code pidautotuner-ftc} JitPack dependency.
// */
////@Config //For FtcDash
//@AutoTuneDash
//public class TuningConfig {
//
//    private TuningConfig() {}
//
//    // ---- Hardware -------------------------------------------------------
//
//    /** Hardware configuration name of the motor (e.g. "shooter", "arm"). */
//    public static String MOTOR_NAME = "motor";
//
//    /** Hardware configuration name of the second motor for the dual-velocity tuner. */
//    public static String MOTOR_NAME_2 = "motor2";
//
//    /** Set true if positive power should produce a decreasing encoder reading. */
//    public static boolean REVERSED = false;
//
//    /** Direction of the second motor. Usually true for opposing-face flywheel setups. */
//    public static boolean REVERSED_2 = true;
//
//    /**
//     * True if both motors have encoders connected. False if only motor 1 has
//     * an encoder -- motor 2 is still driven but its velocity is not measured.
//     */
//    public static boolean DUAL_ENCODERS = false;
//
//    // ---- Relay test -----------------------------------------------------
//
//    /**
//     * Bang-bang output magnitude (0 to 1). Start at 0.3–0.4 for arms and lifts
//     * to avoid hitting hard stops; 0.5–0.7 is typical for flywheels.
//     * Increase if the mechanism fails to oscillate.
//     */
//    public static double RELAY_AMPLITUDE = 0.5;
//
//    /** Number of full oscillation cycles to average for Ku and Tu. */
//    public static int CYCLES_TO_COLLECT = 6;
//
//    /** Number of initial settling cycles to discard before averaging. */
//    public static int CYCLES_TO_IGNORE = 2;
//
//    /** Relay test safety timeout in seconds. */
//    public static double RELAY_TEST_TIMEOUT_S = 15.0;
//
//    // ---- Position tuner -------------------------------------------------
//
//    /** Target offset from the motor's start position in encoder ticks. */
//    public static double POSITION_TARGET_TICKS = 400;
//
//    /** Relay deadband for the position tuner in encoder ticks. */
//    public static double POSITION_HYSTERESIS_TICKS = 10;
//
//    // ---- Velocity / PIDF tuner ------------------------------------------
//
//    /**
//     * Encoder ticks per revolution of the OUTPUT shaft (after any gearing).
//     *
//     * Common values:
//     *   GoBILDA 435 RPM  ->  384.5
//     *   GoBILDA 312 RPM  ->  537.7
//     *   GoBILDA 223 RPM  ->  751.8
//     *   REV HD Hex bare  ->   28.0
//     *   NeveRest 40      -> 1120.0
//     *
//     * Used in the conversion: RPM = (velocity * 60.0) / TICKS_PER_REV
//     */
//    public static double TICKS_PER_REV = 28.0;
//
//    /**
//     * Target speed for the velocity tuner in RPM.
//     * Set this to the RPM you want to tune around (e.g. your shooting speed).
//     * Converted to ticks/sec internally using TICKS_PER_REV:
//     *   ticks/sec = (TARGET_RPM * TICKS_PER_REV) / 60.0
//     */
//    public static double TARGET_RPM = 2800;
//
//    /**
//     * Relay deadband for the velocity tuner in RPM.
//     * Converted to ticks/sec internally using TICKS_PER_REV.
//     */
//    public static double VELOCITY_HYSTERESIS_RPM = 30;
//
//    /** Power levels for the feedforward (kF) characterisation sweep. */
//    public static double[] FEEDFORWARD_TEST_POWERS = {0.5, 0.75, 1.0};
//
//    /** Time in seconds to hold each feedforward power level before sampling. */
//    public static double FEEDFORWARD_SETTLE_TIME_S = 1.5;
//
//    /**
//     * False = PD-only Ziegler-Nichols rules with kI = 0 (recommended for
//     * flywheels where kF handles steady-state error and integral causes windup).
//     * True = full PID rule family including integral terms.
//     */
//    public static boolean TUNE_INTEGRAL_TERM = false;
//
//    // ---- Helpers --------------------------------------------------------
//
//    /**
//     * Returns the target velocity in ticks/sec, converted from TARGET_RPM.
//     *
//     * Formula: ticks/sec = (RPM * TICKS_PER_REV) / 60.0
//     * (inverse of: RPM = (velocity * 60.0) / TICKS_PER_REV)
//     */
//    public static double targetTicksPerSec() {
//        return (TARGET_RPM * TICKS_PER_REV) / 60.0;
//    }
//
//    /**
//     * Returns the relay hysteresis in ticks/sec, converted from VELOCITY_HYSTERESIS_RPM.
//     */
//    public static double hysteresisTicksPerSec() {
//        return (VELOCITY_HYSTERESIS_RPM * TICKS_PER_REV) / 60.0;
//    }
//
//    /**
//     * Converts a velocity in ticks/sec to RPM.
//     * Formula: RPM = (velocity * 60.0) / TICKS_PER_REV
//     */
//    public static double toRPM(double ticksPerSec) {
//        return (ticksPerSec * 60.0) / TICKS_PER_REV;
//    }
//}