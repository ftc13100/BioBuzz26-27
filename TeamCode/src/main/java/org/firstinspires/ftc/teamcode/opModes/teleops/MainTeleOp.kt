package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.follower.ManualDrive
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.Pose
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot
import org.firstinspires.ftc.teamcode.utils.BiLinearShooter
import org.firstinspires.ftc.teamcode.utils.LogTest
import org.firstinspires.ftc.teamcode.utils.PoseStorage
import org.firstinspires.ftc.teamcode.utils.ZoneDetection
import kotlin.math.abs

private const val TELEMETRY_INTERVAL: Int = 250
private var firstOnUpdate = true
private var lastLoopTime = 0.0
private var maxLoopTime = 0.0
private var loopTimeAverage = 0.0
private var lastTelemetryTime = 0.0

@NextTeleop
class MainTeleop(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {

    private lateinit var logger: LogTest
    private var driveScalar = 1.0
    private var pathCommand: Command? = null

    override fun start() {
        beaverRobot.follower.setPose(PoseStorage.autonomousEndPose)
        logger = LogTest()

        val driver = CommandGamepad(gamepad1)
        val operator = CommandGamepad(gamepad2)

        driver.y.toggleOnTrue(instant { driveScalar = 0.5 })
        
        // Automated Path
        driver.a.onTrue(instant {
            val startPose = beaverRobot.follower.pose()
            var targetPose = Pose(39.0, 32.0, 0.0)
            if (PoseStorage.blueAlliance) {
                targetPose = PoseStorage.mirror(targetPose)
            }
            
            val path = com.pedropathing.api.Paths.line(startPose, targetPose)
            pathCommand = PedroCommands.follow(beaverRobot.follower, path)
            pathCommand?.schedule()
        })

        driver.rightBumper.toggleOnTrue(beaverRobot.intake.intake())
        driver.leftBumper.toggleOnTrue(beaverRobot.intake.outtake())

        operator.a.toggleOnTrue(beaverRobot.spindexer.spinShotIndex())
        operator.x.onTrue(beaverRobot.spindexer.autoIndex(0))
        operator.y.onTrue(beaverRobot.spindexer.autoIndex(1))
        operator.b.onTrue(beaverRobot.spindexer.autoIndex(2))
        
        operator.leftStickButton.onTrue(instant {
            if (beaverRobot.turret.goalTrackingActive) {
                beaverRobot.turret.stopTracking()
            } else {
                beaverRobot.turret.trackTarget()
                beaverRobot.shooter.manualOffset = 0.0
                beaverRobot.shooterAngle.manualOffset = 0.0
            }
        })

        driver.leftTrigger.isOver(0.5).and(driver.rightTrigger.isOver(0.5)).onTrue(instant {
            PoseStorage.blueAlliance = !PoseStorage.blueAlliance
        })
    }

    override fun periodic() {
        val now = System.nanoTime() / 1_000_000.0

        if (firstOnUpdate) {
            lastTelemetryTime = now
            lastLoopTime = now
            firstOnUpdate = false
            return
        }

        val loopTime = now - lastLoopTime
        lastLoopTime = now
        loopTimeAverage = loopTimeAverage * 0.95 + loopTime * 0.05
        if (loopTime > maxLoopTime) maxLoopTime = loopTime

        // Joystick Interruption of automated paths
        if (pathCommand?.isScheduled == true || beaverRobot.follower.isBusy()) {
            if (abs(gamepad1.left_stick_y) > 0.1 || abs(gamepad1.left_stick_x) > 0.1 || abs(gamepad1.right_stick_x) > 0.1) {
                pathCommand?.cancel()
                beaverRobot.follower.stop()
            }
        }

        if (beaverRobot.turret.goalTrackingActive) {
            val shot = BiLinearShooter.getShot(
                beaverRobot.turret.turretX,
                beaverRobot.turret.turretY,
                beaverRobot.follower.velocity().toVector()
            )
            BiLinearShooter.applyShot(shot, beaverRobot)

            val inZone = ZoneDetection.poseInTriangle(beaverRobot.follower.pose(), ZoneDetection.scaledCloseShootingZone) ||
                         ZoneDetection.poseInTriangle(beaverRobot.follower.pose(), ZoneDetection.scaledFarShootingZone)
            
            if (inZone && beaverRobot.turret.targetReached && !beaverRobot.intake.isRunning) {
                if (!beaverRobot.spindexer.isBusy) {
                    beaverRobot.spindexer.spinShot().schedule()
                }
            } else if (!beaverRobot.spindexer.atIntakePos && !beaverRobot.spindexer.isBusy) {
                beaverRobot.spindexer.stop().schedule()
                beaverRobot.spindexer.toIntakePos().schedule()
            }
        }

        ManualDrive.driveOrHold(
            beaverRobot.follower,
            -gamepad1.left_stick_y.toDouble() * driveScalar,
            -gamepad1.left_stick_x.toDouble() * driveScalar,
            -gamepad1.right_stick_x.toDouble() * driveScalar
        )
        
        beaverRobot.updateFollower()

        if (now - lastTelemetryTime > TELEMETRY_INTERVAL) {
            lastTelemetryTime = now
            
            telemetry.addData("Loop Hz", "%.2f".format(1000.0 / loopTime))
            telemetry.addData("Alliance", if (PoseStorage.blueAlliance) "BLUE" else "RED")
            telemetry.addData("Robot Pos", beaverRobot.follower.pose())
            telemetry.addData("Turret Pos", "(%.1f, %.1f)".format(beaverRobot.turret.turretX, beaverRobot.turret.turretY))
            telemetry.addData("Shooter RPM", beaverRobot.shooter.motor.encoderVelocity.magnitude)
            telemetry.addData("Tracking Active", beaverRobot.turret.goalTrackingActive)
            telemetry.update()

            logger.log(
                System.currentTimeMillis(),
                beaverRobot.follower.pose().x(),
                beaverRobot.follower.pose().y(),
                beaverRobot.follower.pose().heading(),
                BiLinearShooter.projectedX,
                BiLinearShooter.projectedY,
                beaverRobot.turret.targetAngleField,
                beaverRobot.turret.targetAngleRobotRef,
                beaverRobot.shooter.targetVelocity,
                beaverRobot.shooter.motor.encoderVelocity.magnitude,
                beaverRobot.shooter.motor.throttle
            )
        }
    }

    override fun end() {
        logger.close()
        beaverRobot.turret.stopTracking()
        beaverRobot.shooter.stop().schedule()
    }
}
