package org.firstinspires.ftc.teamcode.opmodes.teleop
import com.pedropathing.ivy.*
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.parallel
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot
import org.firstinspires.ftc.teamcode.core.OctoquadPorts
import org.firstinspires.ftc.teamcode.utils.BiLinearShooter
import org.firstinspires.ftc.teamcode.utils.BeaverLogger
import org.firstinspires.ftc.teamcode.core.PoseManager

private const val TELEMETRY_INTERVAL: Int = 250
private var firstOnUpdate = true
private var lastLoopTime = 0.0
private var maxLoopTime = 0.0
private var minLoopTime = Double.MAX_VALUE
private var lastTelemetryTime = 0.0
private var startSystemTime = 0.toLong()

@NextTeleop("MainTeleOp")
class MainTeleOp(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {
    private lateinit var logger: BeaverLogger
    private var driveScalar = 1.0

    override fun start() {
        beaverRobot.follower.setPose(PoseManager.EndPose)
        logger = BeaverLogger()
        buildDriverControls()
        beaverRobot.turret.stopTracking()
        beaverRobot.turret.toAngle(180.0)
        beaverRobot.ramp.lift()
        beaverRobot.gate.close()
        startSystemTime = System.currentTimeMillis()
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
        if (loopTime > maxLoopTime) maxLoopTime = loopTime
        if (loopTime < minLoopTime) minLoopTime = loopTime

        beaverRobot.octoquadReader.readAll()

        if (beaverRobot.follower.pose().y() > 72.0) beaverRobot.hiveManager.hiveOnRight = false
        else beaverRobot.hiveManager.hiveOnRight = true

        if (beaverRobot.turret.goalTrackingActive) {
            val shot = BiLinearShooter.getShot(
                if (beaverRobot.hiveManager.hiveOnRight) beaverRobot.turret.turretX else -beaverRobot.turret.turretX,
                beaverRobot.turret.turretY,
                beaverRobot.follower.velocity().toVector(),
                beaverRobot.hiveManager.hiveOnRight
            )
            BiLinearShooter.applyShot(shot, beaverRobot)
        }

        beaverRobot.updateFollower()
        if (now - lastTelemetryTime > TELEMETRY_INTERVAL) {
            lastTelemetryTime = now

            telemetry.addData("Robot Pos", beaverRobot.follower.pose())
            telemetry.addData("Turret Pos", "(%.1f, %.1f)".format(beaverRobot.turret.turretX, beaverRobot.turret.turretY))

            telemetry.addData("Shooter RPM", beaverRobot.octoquadReader.getVelocity(OctoquadPorts.Q_SHOOTER.channel))
            telemetry.addData("Tracking Active", beaverRobot.turret.goalTrackingActive)

            telemetry.addData("Loop Time", "%.2f ms".format(loopTime))
            telemetry.addData("Min Loop Time", "%.2f ms".format(minLoopTime))
            telemetry.addData("Max Loop Time", "%.2f ms".format(maxLoopTime))
            telemetry.addData("Target Pos", beaverRobot.hiveManager.getTargetPose())

            telemetry.addData("Shooter", beaverRobot.shooter.targetVelocity + beaverRobot.shooter.manualOffset)
            telemetry.addData("Hood", beaverRobot.turretHood.targetPosition + beaverRobot.turretHood.manualOffset)

            telemetry.addLine(PoseManager.resetPoses.toString())
            telemetry.update()

            logger.log(
                "time" to System.currentTimeMillis() - startSystemTime,
                "posX" to beaverRobot.follower.pose().x(),
                "posY" to beaverRobot.follower.pose().y(),
                "heading" to beaverRobot.follower.pose().heading(),
                "shooterTarget" to beaverRobot.shooter.targetVelocity,
                "shooterActual" to beaverRobot.shooter.motor.encoderVelocity.magnitude,
                "shooterThrottle" to beaverRobot.shooter.motor.throttle
            )
        }

    }

    override fun end() {
        PoseManager.EndPose = beaverRobot.follower.pose()
        logger.close()
        beaverRobot.turret.stopTracking()
        beaverRobot.shooter.stop().schedule()
    }

    fun buildDriverControls() {
        val driver = CommandGamepad(gamepad1)
        val operator = CommandGamepad(gamepad2)

        // Driver Controls
        beaverRobot.drivetrain.startDrivetrain(gamepad1)

        driver.rightBumper.toggleOnTrue(beaverRobot.intake.intake())

        driver.leftBumper.toggleOnTrue(beaverRobot.intake.outtake())

        driver.a.onTrue(instant { beaverRobot.follower.setPose(PoseManager.masterResetPose) })

        driver.rightTrigger.isOver(0.5)
            .toggleOnTrue(instant { beaverRobot.follower.setPose(PoseManager.getResetPose(beaverRobot.follower.pose())) })

        driver.leftTrigger.isOver(0.2)
            .toggleOnTrue(parallel(
                beaverRobot.transfer.shoot(),
                beaverRobot.gate.open()
            ))
            .toggleOnFalse(beaverRobot.gate.close())

        driver.a.toggleOnTrue(instant { beaverRobot.ramp.switchPosition() })

        driver.y.onTrue(instant { driveScalar = 0.5 })
        driver.y.onFalse(instant {driveScalar = 1.0})

        // Operator Controls
        operator.dpadUp.onTrue(instant { beaverRobot.shooter.manualOffset += 10 })
        operator.dpadDown.onTrue(instant { beaverRobot.shooter.manualOffset -= 10 })
        operator.dpadRight.onTrue(instant { beaverRobot.turretHood.manualOffset += .05 })
        operator.dpadLeft.onTrue(instant { beaverRobot.turretHood.manualOffset -= .05 })
    }
}
