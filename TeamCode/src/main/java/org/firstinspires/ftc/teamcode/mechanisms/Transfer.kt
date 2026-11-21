package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.RobotHardware
import kotlin.math.abs

class Transfer : Mechanism {
    val motor = NextMotor(RobotHardware.M_TRANSFER.deviceName)

    val isRunning: Boolean
        get() = abs(motor.throttle) > 0.1

    init {
        motor.direction = NextMotor.Direction.REVERSE
    }

    fun outtake(): Command = Command.build()
        .setStart { motor.throttle = -1.0 }
        .setEnd { _ -> motor.throttle = 0.0 }
        .requiring(this)

    fun shoot(): Command = Command.build()
        .setStart { motor.throttle = 1.0 }
        .setEnd { _ -> motor.throttle = 0.0 }
        .requiring(this)

    fun stop(): Command = instant { motor.throttle = 0.0 }
}