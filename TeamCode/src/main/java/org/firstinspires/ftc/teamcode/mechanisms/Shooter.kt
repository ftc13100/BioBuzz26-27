package org.firstinspires.ftc.teamcode.mechanisms

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.OctoquadPorts
import org.firstinspires.ftc.teamcode.core.RobotHardware
import org.firstinspires.ftc.teamcode.utils.VoltageCompensatingMotor
import kotlin.math.abs

@Configurable
class Shooter(val octoquadReader: OctoquadReader = OctoquadReader()) : Mechanism {
    val motor = NextMotor(RobotHardware.M_SHOOTER.deviceName)
    val compensatedMotor = VoltageCompensatingMotor(motor, nominalVoltage = 12.5)
    val MAX_SPEED = 2000.0

    companion object {
        @JvmField var useVoltageComp = true
        @JvmField var targetVelocity = 0.0
        @JvmField var manualOffset = 0.0

        @JvmField var kP = 0.0026
        @JvmField var kD = 0.000
        @JvmField var kS = 0.155
        @JvmField var kV = 0.000485
        @JvmField var kA = 0.000
        @JvmField var nominalVoltage = 12.5
    }

    var useVoltageComp: Boolean
        get() = Shooter.useVoltageComp
        set(value) {
            Shooter.useVoltageComp = value
            compensatedMotor.enabled = value
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

    var kD: Double
        get() = Shooter.kD
        set(value) { Shooter.kD = value }

    var kS: Double
        get() = Shooter.kS
        set(value) { Shooter.kS = value }

    var kV: Double
        get() = Shooter.kV
        set(value) { Shooter.kV = value }

    var kA: Double
        get() = Shooter.kA
        set(value) { Shooter.kA = value }

    var active = false
    var ready = false
    private val timer = ElapsedTime()

    init {
        motor.direction = NextMotor.Direction.REVERSE
        motor.zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT
    }

    override fun periodic() {
        compensatedMotor.enabled = Shooter.useVoltageComp

        motor.velocityConstants.apply {
            this.kP = Shooter.kP
            this.kD = Shooter.kD
            this.kS = Shooter.kS
            this.kV = Shooter.kV
            this.kA = Shooter.kA
        }

        if (active) {
            val currentVel = abs(octoquadReader.getVelocity(OctoquadPorts.Q_SHOOTER.channel))
            val error = targetVelocity - currentVel

            val rawPower = if (error > 100) {
                1.0
            } else if (error < - 100) {
                0.0
            } else {
                val pidPower = motor.velocityPID.calculate(
                    reference = targetVelocity,
                    measured = currentVel
                )
                val ffPower = motor.velocityFF.calculate(targetVelocity)
                pidPower + ffPower
            }

            compensatedMotor.setPower(rawPower)

            if (abs(error) < 21.0 && !ready) {
                ready = true
            }
        } else {
            compensatedMotor.setPower(0.0)
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
