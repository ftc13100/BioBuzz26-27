//TODO: Fix defaulting to ramp up
package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import dev.nextftc.hardware.actuators.NextServo
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.RobotHardware

class Ramp : Mechanism {
    val servo = NextServo(RobotHardware.S_RAMP.deviceName)
    var up = true

    fun lift() {
        servo.position = 1.0
        up = true
    }
    fun deploy() {
        servo.position = 0.62
        up = false
    }

    fun switchPosition() {
        if (up) { deploy() }
        else { lift() }
    }
}