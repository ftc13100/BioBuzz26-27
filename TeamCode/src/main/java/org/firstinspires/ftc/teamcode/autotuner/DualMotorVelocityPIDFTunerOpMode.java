//package org.firstinspires.ftc.teamcode.autotuner;
//
//import com.aaravdhawan25.pidautotuner.ftc.DualMotorPIDMaster;
//
//import com.aaravdhawan25.pidautotuner.ftc.dashboard.AutoTuneTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
///**
// * Auto-tunes PIDF gains for a dual-motor velocity-controlled mechanism
// * (e.g. a two-flywheel shooter) using the relay-feedback method.
// * Both motors are driven with the same output -- the resulting gains
// * are applied identically to both.
// *
// * <p>Set {@link TuningConfig#TARGET_RPM}, {@link TuningConfig#TICKS_PER_REV},
// * both motor names, and directions in {@link TuningConfig}. Everything
// * else runs automatically.
// *
// * <h2>Setup</h2>
// * <ol>
// *     <li>In {@link TuningConfig}, set:
// *         <ul>
// *             <li>{@code MOTOR_NAME} + {@code REVERSED} for motor 1</li>
// *             <li>{@code MOTOR_NAME_2} + {@code REVERSED_2} for motor 2</li>
// *             <li>{@code TARGET_RPM} to your target flywheel speed</li>
// *             <li>{@code TICKS_PER_REV} for your motor</li>
// *             <li>{@code DUAL_ENCODERS = false} if only motor 1 has an encoder</li>
// *         </ul>
// *     </li>
// *     <li>Run this OpMode and press Start.</li>
// *     <li><b>Phase 1:</b> both motors oscillate around TARGET_RPM automatically.</li>
// *     <li><b>Phase 2:</b> feedforward sweep computes kF on both motors.</li>
// *     <li>Read candidate gains from telemetry -- apply identically to both motors.</li>
// *     <li>Hold gamepad1.A to live-test on both motors simultaneously.</li>
// * </ol>
// */
//@TeleOp(name = "Dual Velocity PIDF Tuner", group = "FtcAutoTune")
//public class DualMotorVelocityPIDFTunerOpMode extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
////        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
//        telemetry = new AutoTuneTelemetry(hardwareMap,telemetry);
//        // Convert RPM target to ticks/sec using: ticks/sec = (RPM * TICKS_PER_REV) / 60.0
//        double targetTicksPerSec = TuningConfig.targetTicksPerSec();
//
//        DualMotorPIDMaster pid = new DualMotorPIDMaster(
//                hardwareMap,
//                TuningConfig.MOTOR_NAME,   TuningConfig.REVERSED,
//                TuningConfig.MOTOR_NAME_2, TuningConfig.REVERSED_2,
//                targetTicksPerSec,
//                TuningConfig.hysteresisTicksPerSec(),
//                TuningConfig.RELAY_AMPLITUDE,
//                TuningConfig.CYCLES_TO_COLLECT,
//                TuningConfig.CYCLES_TO_IGNORE,
//                TuningConfig.RELAY_TEST_TIMEOUT_S,
//                TuningConfig.FEEDFORWARD_TEST_POWERS,
//                TuningConfig.FEEDFORWARD_SETTLE_TIME_S,
//                TuningConfig.TUNE_INTEGRAL_TERM,
//                TuningConfig.DUAL_ENCODERS);
//
//        telemetry.addLine("=== PIDF Auto Tuner (Dual Velocity) ===");
//        telemetry.addData("Motor 1", TuningConfig.MOTOR_NAME + (TuningConfig.REVERSED ? " (reversed)" : ""));
//        telemetry.addData("Motor 2", TuningConfig.MOTOR_NAME_2 + (TuningConfig.REVERSED_2 ? " (reversed)" : ""));
//        telemetry.addData("Encoder mode", TuningConfig.DUAL_ENCODERS ? "DUAL (both motors)" : "SINGLE (motor 1 only)");
//        telemetry.addData("TICKS_PER_REV", TuningConfig.TICKS_PER_REV);
//        telemetry.addData("Target", String.format("%.1f RPM  (%.1f ticks/s)",
//                TuningConfig.TARGET_RPM, targetTicksPerSec));
//        telemetry.addLine("Press START. Both motors will oscillate automatically.");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !pid.isTuningComplete()) {
//            pid.tuningStep(getRuntime());
//            telemetry.clearAll();
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
//            pid.stop();
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
//
//            }
//            telemetry.update();
//        }
//
//        pid.stop();
//    }
//}