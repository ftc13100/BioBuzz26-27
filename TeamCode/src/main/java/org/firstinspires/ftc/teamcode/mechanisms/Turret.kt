package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.follower.Follower
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.hardware.actuators.NextServo
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.RobotHardware
import org.firstinspires.ftc.teamcode.utils.HiveManager
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class Turret(val follower: Follower) : Mechanism {
    val turret1 = NextServo(RobotHardware.S_TURRET1.deviceName)
    val turret2 = NextServo(RobotHardware.S_TURRET2.deviceName)
    val turretDigital = NextMotor(RobotHardware.Q_TURRET.deviceName)

    val LIMIT_LOW = 22.2
    val LIMIT_HIGH = 337.8

    var targetServoPosition = 0.5
    var targetAngleRobotRef: Double = 180.0
    var targetAngleField: Double = 270.0

    @JvmField var goalTrackingActive = false
    @JvmField var TURRET_OFFSET = -2.03852
    @JvmField var manualOffsetAngle: Double = 0.0
    @JvmField var kVF = 0.0

    var turretX = 0.0
    var turretY = 0.0
    var targetReached = false

    override fun periodic() {
        if (!goalTrackingActive) return

        val robotPose = follower.pose()

        // Robot heading in degrees for offset calculation
        var robotHeading = Math.toDegrees(robotPose.heading())
        if (robotHeading < 0.0) robotHeading += 360.0
        val turretRobotAdj = 360.0 - robotHeading

        // Turret field position
        turretX = robotPose.x() + TURRET_OFFSET * cos(robotPose.heading())
        turretY = robotPose.y() + TURRET_OFFSET * sin(robotPose.heading())

        // Calculate dynamic goal based directly on current turret X position
        val goal = HiveManager.getTargetPose(turretX)

        val dx = goal.x() - turretX
        val dy = goal.y() - turretY

        val targetAngleRad = atan2(dy, dx)
        targetAngleField = Math.toDegrees(targetAngleRad)
        if (targetAngleField < 0.0) targetAngleField += 360.0

        val targetAngleRobot = targetAngleField + turretRobotAdj
        toAngle(targetAngleRobot + manualOffsetAngle)
        
        val turretRelativePos = (turretDigital.encoderPosition.magnitude / 12000.0) * 360.0
        targetReached = abs(turretRelativePos - targetAngleRobotRef) < 2.0
    }

    fun toAngle(angle: Double) {
        targetAngleRobotRef = angle % 360.0
        if (targetAngleRobotRef < 0.0) targetAngleRobotRef += 360.0

        val clampedAngle = targetAngleRobotRef.coerceIn(LIMIT_LOW, LIMIT_HIGH)
        targetServoPosition = (clampedAngle - LIMIT_LOW) / (LIMIT_HIGH - LIMIT_LOW)
        
        turret1.position = targetServoPosition
        turret2.position = targetServoPosition
    }

    fun trackTarget() { goalTrackingActive = true }
    fun stopTracking() { goalTrackingActive = false }
}
