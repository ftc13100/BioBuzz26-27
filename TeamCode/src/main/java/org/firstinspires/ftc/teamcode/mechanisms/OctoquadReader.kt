package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad
import dev.nextftc.hardware.RobotController.hardwareMap
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.RobotHardware

class OctoquadReader : Mechanism {
    val octoquad = hardwareMap.get(OctoQuad::class.java, RobotHardware.I2C_OCTOQUAD.deviceName)
    var data = OctoQuad.EncoderDataBlock()

    var velocitySampleIntervalMs = 50

    init {
        try {
            octoquad.setAllVelocitySampleIntervals(velocitySampleIntervalMs)
        } catch (e: Exception) {
            // Ignore init failure if hardware is not yet connected during unit testing
        }
    }

    override fun periodic() {
        data = octoquad.readAllEncoderData()
    }

    fun readAll() {
        data = octoquad.readAllEncoderData()
    }

    fun dataValid(): Boolean { return data.isDataValid() }

    fun getRawVelocity(channel: Int): Short { return data.velocities[channel] }

    fun getVelocity(channel: Int): Double {
        val samplesPerSec = 1000.0 / velocitySampleIntervalMs
        return data.velocities[channel] * samplesPerSec
    }
}
