package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import dev.nextftc.units.radiansPerSecond
import org.firstinspires.ftc.teamcode.core.RobotHardware
import kotlin.math.abs

class Shooter : Mechanism {
    val motor = NextMotor(RobotHardware.M_SHOOTER.deviceName)

    @JvmField var targetVelocity = 0.0
    @JvmField var manualOffset = 0.0
    val MAX_SPEED = 2200.0

    @JvmField var kP = 0.001
    @JvmField var kS = 0.061
    @JvmField var kV = 0.000385

    var active = false
    var ready = false
    private val timer = ElapsedTime()

    init {
        motor.direction = NextMotor.Direction.REVERSE
        motor.zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT
    }

    override fun periodic() {

        motor.velocityConstants.apply {
            this.kP = this@Shooter.kP
            this.kS = this@Shooter.kS
            this.kV = this@Shooter.kV
        }

        if (active) {
            val currentVel = motor.encoderVelocity.magnitude
            val error = targetVelocity - currentVel

            //bang-bang controller
            if (error > 100.0) {
                motor.throttle = 1.0
            } else if (error < -100.0) {
                motor.throttle = 0.0
            } else {
                motor.setVelocitySetpoint(targetVelocity.radiansPerSecond)
            }

            // Check if shooter has stabilized
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
