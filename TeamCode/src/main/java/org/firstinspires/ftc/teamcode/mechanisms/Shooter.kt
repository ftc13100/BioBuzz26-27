package org.firstinspires.ftc.teamcode.mechanisms

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.hardware.RobotController
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.OctoquadPorts
import org.firstinspires.ftc.teamcode.core.RobotHardware
import kotlin.math.abs
import kotlin.math.max

@Configurable
class Shooter(val octoquadReader: OctoquadReader = OctoquadReader()) : Mechanism {
    val motor = NextMotor(RobotHardware.M_SHOOTER.deviceName)
    val MAX_SPEED = 6000.0

    companion object {
        @JvmField var targetVelocity = 0.0
        @JvmField var manualOffset = 0.0

        @JvmField var kP = -0.00002
        @JvmField var kD = 0.000
        @JvmField var kS = 0.29
        @JvmField var kV = 0.0007
        @JvmField var kA = 0.000
        @JvmField var nominalVoltage = 12.0
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

    private var cachedVoltage = 12.0
    private var lastVoltageReadMs = 0L

    val currentVoltage: Double
        get() {
            val now = System.currentTimeMillis()
            if (now - lastVoltageReadMs >= 250L || lastVoltageReadMs == 0L) {
                try {
                    val measured = RobotController.hardwareMap.voltageSensor.firstOrNull()?.voltage ?: 12.0
                    if (measured > 6.0) {
                        cachedVoltage = measured
                    }
                } catch (e: Exception) {
                    cachedVoltage = nominalVoltage
                }
                lastVoltageReadMs = now
            }
            return cachedVoltage
        }

    val voltageCompMultiplier: Double
        get() = nominalVoltage / max(6.0, currentVoltage)

    init {
        motor.direction = NextMotor.Direction.REVERSE
        motor.zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT
    }

    override fun periodic() {
        if (active && targetVelocity > 0) {
            val currentVel = abs(octoquadReader.getVelocity(OctoquadPorts.Q_SHOOTER.channel).toDouble())
            val error = targetVelocity - currentVel

            val ff = Shooter.kS + (Shooter.kV * targetVelocity)
            val p = Shooter.kP * error
            val rawPower = ff + p

            val compensatedPower = rawPower * voltageCompMultiplier
            motor.throttle = compensatedPower.coerceIn(0.0, 1.0)

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
