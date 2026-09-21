package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.follower.ManualDrive
import com.pedropathing.ivy.commands.Commands.instant
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot
import kotlin.math.abs

@NextTeleop(name = "Turret Test & Tune")
class TurretTuner(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {

    private var manualAngle = 180.0

    override fun start() {
        beaverRobot.turret.stopTracking()
        
        val driver = CommandGamepad(gamepad1)

        driver.x.onTrue(instant {
            if (beaverRobot.turret.goalTrackingActive) beaverRobot.turret.stopTracking()
            else beaverRobot.turret.trackTarget()
        })

        driver.dpadRight.onTrue(instant { manualAngle += 20.0 })
        driver.dpadLeft.onTrue(instant { manualAngle -= 20.0 })

        driver.a.onTrue(instant {
            if (!beaverRobot.turret.goalTrackingActive) {
                beaverRobot.turret.toAngle(manualAngle)
            }
        })
    }

    override fun periodic() {
        ManualDrive.driveOrHold(
            beaverRobot.follower,
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
        )
        
        beaverRobot.updateFollower()

        val robotPose = beaverRobot.follower.pose()
        val turretRelPos = (beaverRobot.turret.turretDigital.encoderPosition.magnitude / 12000.0) * 360.0
        
        telemetry.addData("Tracking Active", beaverRobot.turret.goalTrackingActive)
        telemetry.addData("X/Y", "(%.1f, %.1f)".format(robotPose.x(), robotPose.y()))
        telemetry.addData("Turret Pos (Field)", "(%.1f, %.1f)".format(beaverRobot.turret.turretX, beaverRobot.turret.turretY))
        telemetry.addData("Angular Vel", beaverRobot.follower.velocity().omega)
        telemetry.addData("FF (kVF)", beaverRobot.follower.velocity().omega * beaverRobot.turret.kVF)
        telemetry.addData("Target Angle (Robot Ref)", beaverRobot.turret.targetAngleRobotRef)
        telemetry.addData("Encoder Pos", "%.1f".format(turretRelPos))
        telemetry.update()
    }
}
