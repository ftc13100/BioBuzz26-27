package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.follower.ManualDrive
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.math.Pose
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot
import org.firstinspires.ftc.teamcode.core.Dimensions
import org.firstinspires.ftc.teamcode.utils.BiLinearShooter
import org.firstinspires.ftc.teamcode.utils.BeaverLogger
import org.firstinspires.ftc.teamcode.utils.PoseStorage
import kotlin.math.abs

private const val TELEMETRY_INTERVAL: Int = 250
private var firstOnUpdate = true
private var lastLoopTime = 0.0
private var maxLoopTime = 0.0
private var loopTimeAverage = 0.0
private var lastTelemetryTime = 0.0

@NextTeleop("MainTeleOp")
class MainTeleOp(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {
    private lateinit var logger: BeaverLogger
    private var driveScalar = 1.0
    private var pathCommand: Command? = null

    override fun start() {
        beaverRobot.follower.setPose(PoseStorage.autonomousEndPose)
        logger = BeaverLogger()

        buildDriverControls()

        beaverRobot.turret.trackTarget()
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

    fun buildDriverControls() {
        val driver = CommandGamepad(gamepad1)
        val operator = CommandGamepad(gamepad2)

        // Driver Controls
        driver.rightBumper
            .toggleOnTrue(beaverRobot.intake.intake())

        driver.leftBumper
            .toggleOnTrue(beaverRobot.intake.outtake())

        driver.leftTrigger.isOver(0.5)
            .toggleOnTrue(beaverRobot.spindexer.spinShot())
            .toggleOnFalse(beaverRobot.spindexer.stop())
            .toggleOnFalse(beaverRobot.spindexer.toIntakePos())

        driver.rightTrigger.isOver(0.5)
            .toggleOnTrue(instant { beaverRobot.follower.setPose(PoseStorage.resetPose) })

        driver.y.toggleOnTrue(instant { driveScalar = 0.5 })

        // Operator Controls
        operator.a.toggleOnTrue(beaverRobot.spindexer.spinShotIndex())
        operator.x.onTrue(beaverRobot.spindexer.autoIndex(0))
        operator.y.onTrue(beaverRobot.spindexer.autoIndex(1))
        operator.b.onTrue(beaverRobot.spindexer.autoIndex(2))
    }
}
