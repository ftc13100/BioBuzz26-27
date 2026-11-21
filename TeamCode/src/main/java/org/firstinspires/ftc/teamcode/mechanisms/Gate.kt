package org.firstinspires.ftc.teamcode.mechanisms

import dev.nextftc.hardware.actuators.NextServo
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.RobotHardware

class Gate : Mechanism {
    val servo = NextServo(RobotHardware.S_RAMP.deviceName)

    fun open() = instant { servo.position = 85.0 }
    fun close() = instant { servo.position = 95.0 }
}