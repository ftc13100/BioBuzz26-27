package org.firstinspires.ftc.teamcode.mechanisms

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import dev.nextftc.units.radiansPerSecond
import org.firstinspires.ftc.teamcode.core.OctoquadPorts
import org.firstinspires.ftc.teamcode.core.RobotHardware
import kotlin.math.abs

@Configurable
class Shooter(val octoquadReader: OctoquadReader = OctoquadReader()) : Mechanism {
    val motor = NextMotor(RobotHardware.M_SHOOTER.deviceName)
    val MAX_SPEED = 6000.0

    companion object {
        @JvmField var targetVelocity = 0.0
        @JvmField var manualOffset = 0.0

        @JvmField var kP = -0.00002
        @JvmField var kS = 0.24
        @JvmField var kV = 0.00065
    }

    var targetVelocity: Double
        get() = Shooter.targetVelocity
        set(value) { Shooter.targetVelocity = value }

    var manualOffset: Double
        get() = Shooter.manualOffset
        set(value) { Shooter.manualOffset = value }

    var kP: Double
        get() = Shooter.kP
        set(value) { Shooter.kP = value }

    var kS: Double
        get() = Shooter.kS
        set(value) { Shooter.kS = value }

    var kV: Double
        get() = Shooter.kV
        set(value) { Shooter.kV = value }

    var active = false
    var ready = false
    private val timer = ElapsedTime()

    init {
        motor.direction = NextMotor.Direction.REVERSE
        motor.zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT
    }

    override fun periodic() {

        motor.velocityConstants.apply {
            this.kP = Shooter.kP
            this.kS = Shooter.kS
            this.kV = Shooter.kV
        }

        if (active) {
            val currentVel = abs(octoquadReader.getVelocity(OctoquadPorts.Q_SHOOTER.channel).toDouble())
            val error = targetVelocity - currentVel

            //if (error > 500.0) {
            //    motor.throttle = 1.0
            //} else if (error < -500.0) {
            //    motor.throttle = 0.0
            //} else {
                motor.setVelocitySetpoint(targetVelocity.radiansPerSecond)
            //}

            if (abs(error) < 21.0 && !ready) {
                ready = true
            }
        } else {
            motor.throttle = 0.0
            ready = false
        }
    }

    fun spinAtSpeed(speed: Double): Command = Command.build()
        .setStart {
            targetVelocity = speed.coerceIn(0.0, MAX_SPEED)
            active = true
            ready = false
            timer.reset()
        }
        .requiring(this)

    fun stop(): Command = instant {
        active = false
        targetVelocity = 0.0
        ready = false
    }

    fun adjustSpeed(adj: Double) {
        targetVelocity = (targetVelocity + adj).coerceIn(0.0, MAX_SPEED)
    }
}
