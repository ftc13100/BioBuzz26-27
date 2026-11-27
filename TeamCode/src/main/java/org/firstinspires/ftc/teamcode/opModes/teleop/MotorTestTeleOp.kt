package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.ivy.commands.Commands.instant
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot
import org.firstinspires.ftc.teamcode.core.OctoquadPorts
import kotlin.math.abs

@NextTeleop(name = "Playable Motor Test")
class MotorTestTeleOp(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {

    private var shooterSpeed = 0.80
    private var transferSpeed = 0.80
    private var intakeSpeed = 1.00

    private var intakeActive = false
    private var intakeReverse = false

    override fun start() {
        val driver = CommandGamepad(gamepad1)

        // Speed adjustment controls
        driver.dpadUp.onTrue(instant { shooterSpeed = (shooterSpeed + 0.05).coerceIn(0.0, 1.0) })
        driver.dpadDown.onTrue(instant { shooterSpeed = (shooterSpeed - 0.05).coerceIn(0.0, 1.0) })

        driver.dpadRight.onTrue(instant { transferSpeed = (transferSpeed + 0.05).coerceIn(0.0, 1.0) })
        driver.dpadLeft.onTrue(instant { transferSpeed = (transferSpeed - 0.05).coerceIn(0.0, 1.0) })

        driver.y.onTrue(instant { intakeSpeed = (intakeSpeed + 0.05).coerceIn(0.0, 1.0) })
        driver.a.onTrue(instant { intakeSpeed = (intakeSpeed - 0.05).coerceIn(0.0, 1.0) })

        // Intake Toggles
        driver.rightBumper.onTrue(instant {
            intakeActive = !intakeActive
            intakeReverse = false
        })
        driver.leftBumper.onTrue(instant {
            intakeReverse = !intakeReverse
            intakeActive = false
        })

        // Emergency Stop All
        driver.b.onTrue(instant {
            shooterSpeed = 0.0
            transferSpeed = 0.0
            intakeActive = false
            intakeReverse = false
        })
    }

    override fun periodic() {
        // Normal Manual Driving (No Localizer / Position Hold required)
        beaverRobot.follower.manual(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
        )
        beaverRobot.updateFollower()

        beaverRobot.octoquadReader.readAll()

        // Shooter run on Right Trigger
        if (gamepad1.right_trigger > 0.1) {
            beaverRobot.shooter.motor.throttle = shooterSpeed
        } else {
            beaverRobot.shooter.motor.throttle = 0.0
        }

        // Transfer run on Left Trigger
        if (gamepad1.left_trigger > 0.1) {
            beaverRobot.transfer.motor.throttle = transferSpeed
        } else {
            beaverRobot.transfer.motor.throttle = 0.0
        }

        // Intake run on Bumpers
        when {
            intakeActive -> beaverRobot.intake.motor.throttle = intakeSpeed
            intakeReverse -> beaverRobot.intake.motor.throttle = -intakeSpeed
            else -> beaverRobot.intake.motor.throttle = 0.0
        }

        val shooterRpm = abs(beaverRobot.octoquadReader.getVelocity(OctoquadPorts.Q_SHOOTER.channel))

        telemetry.addData("Shooter", "Speed: %.2f | RPM: %.0f (Hold RT)", shooterSpeed, shooterRpm)
        telemetry.addData("Transfer", "Speed: %.2f (Hold LT)", transferSpeed)
        telemetry.addData("Intake", "Speed: %.2f | State: %s (RB/LB)", intakeSpeed, if (intakeActive) "FWD" else if (intakeReverse) "REV" else "OFF")
        telemetry.addLine()
        telemetry.addLine("Controls:")
        telemetry.addLine("  Drive: Left Stick (Translation), Right Stick (Rotation)")
        telemetry.addLine("  Shooter Speed: D-Pad Up/Down")
        telemetry.addLine("  Transfer Speed: D-Pad Right/Left")
        telemetry.addLine("  Intake Speed: Y/A Buttons")
        telemetry.addLine("  Stop All: B Button")
        telemetry.update()
    }
}
