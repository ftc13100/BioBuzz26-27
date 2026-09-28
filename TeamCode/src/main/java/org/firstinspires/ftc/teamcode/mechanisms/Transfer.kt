package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.RobotHardware
import kotlin.math.abs

class Transfer : Mechanism {
    val transferMotor = NextMotor(RobotHardware.M_TRANSFER.deviceName)

    val transferSpeed = 1.0

    val isRunning: Boolean
        get() = abs(transferMotor.throttle) > 0.1

    init {
        transferMotor.direction = NextMotor.Direction.FORWARD
        transferMotor.zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE
    }

    fun spinForward(): Command = Command.build()
        .setStart { transferMotor.throttle = transferSpeed }
        .setEnd { _ -> transferMotor.throttle = 0.0 }
        .requiring(this)

    fun spinReverse(): Command = Command.build()
        .setStart { transferMotor.throttle = -transferSpeed }
        .setEnd { _ -> transferMotor.throttle = 0.0 }
        .requiring(this)


    fun stop(): Command = instant { transferMotor.throttle = 0.0 }

    fun spinFeed(): Command = instant {
        transferMotor.throttle = transferSpeed
    }.requiring(this)

}
