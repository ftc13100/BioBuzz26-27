//package org.firstinspires.ftc.teamcode.autotune;
//
//import com.aaravdhawan25.pidautotuner.ftc.PIDMaster;
//
//import com.aaravdhawan25.pidautotuner.ftc.dashboard.AutoTuneTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
///**
// * Auto-tunes PIDF gains for a velocity-controlled mechanism (flywheel,
// * intake, etc.) using the relay-feedback method.
// *
// * <p>Set {@link TuningConfig#TARGET_RPM} to your target speed in RPM and
// * {@link TuningConfig#TICKS_PER_REV} to your motor's ticks per revolution.
// * Everything else runs automatically.
// *
// * <h2>Setup</h2>
// * <ol>
// *     <li>Set {@code MOTOR_NAME}, {@code TARGET_RPM}, and {@code TICKS_PER_REV}
// *         in {@link TuningConfig}. Verify {@code REVERSED} is correct.</li>
// *     <li>Run this OpMode and press Start.</li>
// *     <li><b>Phase 1 (relay test):</b> motor oscillates around TARGET_RPM automatically.</li>
// *     <li><b>Phase 2 (feedforward sweep):</b> motor ramps through power levels to compute kF.</li>
// *     <li>Read the candidate gain sets from telemetry.</li>
// *     <li>Hold gamepad1.A to live-test the classic ZN candidate.</li>
// * </ol>
// */
//@TeleOp(name = "Velocity PIDF Tuner", group = "FtcAutoTune")
//public class VelocityPIDFTunerOpMode extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
////        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry); // for Ftc Dash
//        telemetry = new AutoTuneTelemetry(hardwareMap,telemetry); // For Pulse
//        // Convert RPM target to ticks/sec using: ticks/sec = (RPM * TICKS_PER_REV) / 60.0
//        double targetTicksPerSec = TuningConfig.targetTicksPerSec();
//
//        PIDMaster pid = new PIDMaster(
//                hardwareMap, TuningConfig.MOTOR_NAME, TuningConfig.REVERSED, false,
//                targetTicksPerSec, TuningConfig.hysteresisTicksPerSec(),
//                TuningConfig.RELAY_AMPLITUDE, TuningConfig.CYCLES_TO_COLLECT, TuningConfig.CYCLES_TO_IGNORE,
//                TuningConfig.RELAY_TEST_TIMEOUT_S, TuningConfig.FEEDFORWARD_TEST_POWERS,
//                TuningConfig.FEEDFORWARD_SETTLE_TIME_S, TuningConfig.TUNE_INTEGRAL_TERM,
//                TuningConfig.TICKS_PER_REV);
//
//        telemetry.addLine("=== PIDF Auto Tuner (Velocity) ===");
//        telemetry.addData("Motor", TuningConfig.MOTOR_NAME);
//        telemetry.addData("TICKS_PER_REV", TuningConfig.TICKS_PER_REV);
//        telemetry.addData("Target", String.format("%.1f RPM  (%.1f ticks/s)",
//                TuningConfig.TARGET_RPM, targetTicksPerSec));
//        telemetry.addLine("Press START. Phase 1: relay test (automatic).");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !pid.isTuningComplete()) {
//            pid.tuningStep(getRuntime());
//            for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
//            telemetry.update();
//        }
//
//        if (pid.timedOut() || !pid.isTuningSuccessful()) {
//            while (opModeIsActive()) {
//                telemetry.clearAll();
//                for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
//                telemetry.update();
//            }
//            return;
//        }
//
//        while (opModeIsActive()) {
//            for (String line : pid.getResultTelemetryLines()) telemetry.addLine(line);
//            if (gamepad1.a) {
//                telemetry.addLine();
//                for (String line : pid.liveTestStep(getRuntime())) telemetry.addLine(line);
//            } else {
//                pid.stopLiveTest();
//            }
//            telemetry.update();
//        }
//    }
//}