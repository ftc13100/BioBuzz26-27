package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import dev.nextftc.hardware.actuators.NextServo
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.RobotHardware

class ShooterAngle : Mechanism {
    val servo = NextServo(RobotHardware.S_TURRET_HOOD.deviceName)

    var targetPosition = 0.0
    var manualOffset = 0.0
    
    val ANGLE_MIN = 0.0
    val ANGLE_MAX = 0.8

    fun update(): Command = instant {
        servo.position = (targetPosition + manualOffset).coerceIn(ANGLE_MIN, ANGLE_MAX)
    }

    fun toPos(pos: Double): Command = instant {
        targetPosition = pos
        servo.position = (targetPosition + manualOffset).coerceIn(ANGLE_MIN, ANGLE_MAX)
    }

    fun angleUp() = toPos(ANGLE_MAX)
    fun angleDown() = toPos(ANGLE_MIN)
    fun angleMid() = toPos(0.65)
    fun angleVeryMid() = toPos(0.58)

    fun adjustAngle(adj: Double) {
        targetPosition = (targetPosition + adj).coerceIn(ANGLE_MIN, ANGLE_MAX)
        servo.position = (targetPosition + manualOffset).coerceIn(ANGLE_MIN, ANGLE_MAX)
    }
}
