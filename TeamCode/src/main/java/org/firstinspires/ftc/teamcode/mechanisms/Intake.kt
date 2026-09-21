package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.RobotHardware
import kotlin.math.abs

class Intake : Mechanism {
    val intakeMotor = NextMotor(RobotHardware.M_INTAKE.deviceName)

    val isRunning: Boolean
        get() = abs(intakeMotor.throttle) > 0.1

    init {
        intakeMotor.direction = NextMotor.Direction.REVERSE
    }

    fun intake(): Command = Command.build()
        .setStart { intakeMotor.throttle = 1.0 }
        .setEnd { _ -> intakeMotor.throttle = 0.0 }
        .requiring(this)

    fun outtake(): Command = Command.build()
        .setStart { intakeMotor.throttle = -1.0 }
        .setEnd { _ -> intakeMotor.throttle = 0.0 }
        .requiring(this)

    fun stop(): Command = instant { intakeMotor.throttle = 0.0 }
}
