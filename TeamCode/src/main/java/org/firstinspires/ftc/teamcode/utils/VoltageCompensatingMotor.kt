package org.firstinspires.ftc.teamcode.utils

import dev.nextftc.hardware.RobotController
import dev.nextftc.hardware.actuators.NextMotor
import kotlin.math.max
import kotlin.time.ComparableTimeMark
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds
import kotlin.time.TimeSource.Monotonic.markNow

class VoltageCompensatingMotor @JvmOverloads constructor(
    val motor: NextMotor,
    private val voltageCacheTime: Duration = 0.25.seconds,
    private val nominalVoltage: Double = 12.5,
    var enabled: Boolean = false
) {
    private val voltageSensor by lazy { RobotController.hardwareMap.voltageSensor.first() }

    private var cachedVoltage: Double = nominalVoltage
    private lateinit var lastVoltageRead: ComparableTimeMark

    val voltage: Double
        get() {
            val currentTime = markNow()
            if (!::lastVoltageRead.isInitialized || currentTime - lastVoltageRead >= voltageCacheTime) {
                try {
                    val measured = voltageSensor.voltage
                    if (measured > 6.0) cachedVoltage = measured
                } catch (e: Exception) {
                    cachedVoltage = nominalVoltage
                }
                lastVoltageRead = currentTime
            }
            return max(6.0, cachedVoltage)
        }

    fun setPower(rawPower: Double) {
        val multiplier = if (enabled) (nominalVoltage / voltage) else 1.0
        val compensated = rawPower * multiplier
        motor.throttle = compensated.coerceIn(-1.0, 1.0)
    }
}
