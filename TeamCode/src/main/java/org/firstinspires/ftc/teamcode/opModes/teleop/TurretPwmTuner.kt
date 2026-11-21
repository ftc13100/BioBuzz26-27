package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.ivy.commands.Commands.instant
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot

@NextTeleop(name = "Turret PWM Control")
class TurretPwmTuner(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {

    private val minPwmUs = 500.0
    private val maxPwmUs = 2500.0
    private var targetPwmUs = 1500.0

    override fun start() {
        beaverRobot.turret.stopTracking()

        // Configure Axon MAX PWM range (500us - 2500us)
        beaverRobot.turret.servo.setPwmRange(minPwmUs, maxPwmUs)

        // Set to middle PWM (1500us)
        targetPwmUs = 1500.0
        applyPwm(targetPwmUs)

        val driver = CommandGamepad(gamepad1)

        // Coarse adjust: +/- 50 microseconds
        driver.dpadUp.onTrue(instant {
            targetPwmUs = (targetPwmUs + 50.0).coerceIn(minPwmUs, maxPwmUs)
        })
        driver.dpadDown.onTrue(instant {
            targetPwmUs = (targetPwmUs - 50.0).coerceIn(minPwmUs, maxPwmUs)
        })

        // Fine adjust: +/- 10 microseconds
        driver.dpadRight.onTrue(instant {
            targetPwmUs = (targetPwmUs + 10.0).coerceIn(minPwmUs, maxPwmUs)
        })
        driver.dpadLeft.onTrue(instant {
            targetPwmUs = (targetPwmUs - 10.0).coerceIn(minPwmUs, maxPwmUs)
        })

        // Reset to middle PWM (1500us)
        driver.a.onTrue(instant {
            targetPwmUs = 1500.0
        })
    }

    override fun periodic() {
        applyPwm(targetPwmUs)

        val normalizedPosition = (targetPwmUs - minPwmUs) / (maxPwmUs - minPwmUs)

        telemetry.addData("Target PWM", "%.1f µs (Middle = 1500.0 µs)".format(targetPwmUs))
        telemetry.addData("Normalized Position", "%.4f".format(normalizedPosition))
        telemetry.addData("PWM Limits", "%.1f µs - %.1f µs".format(minPwmUs, maxPwmUs))
        telemetry.addLine()
        telemetry.addLine("Controls:")
        telemetry.addLine("  D-Pad Up / Down: Coarse adjust (+/- 50 µs)")
        telemetry.addLine("  D-Pad Right / Left: Fine adjust (+/- 10 µs)")
        telemetry.addLine("  A Button: Reset to middle (1500 µs)")
        telemetry.update()
    }

    private fun applyPwm(pwmUs: Double) {
        val normalized = (pwmUs.coerceIn(minPwmUs, maxPwmUs) - minPwmUs) / (maxPwmUs - minPwmUs)
        beaverRobot.turret.servo.position = normalized
    }
}
