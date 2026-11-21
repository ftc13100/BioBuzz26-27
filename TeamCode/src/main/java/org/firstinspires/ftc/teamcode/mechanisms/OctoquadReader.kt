package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad
import dev.nextftc.hardware.RobotController.hardwareMap
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.OctoquadPorts
import org.firstinspires.ftc.teamcode.core.RobotHardware

class OctoquadReader : Mechanism {
    val octoquad = hardwareMap.get(OctoQuad::class.java, RobotHardware.I2C_OCTOQUAD.deviceName)
    var data = OctoQuad.EncoderDataBlock()

    fun readAll() {
        data = octoquad.readAllEncoderData()
    }

    fun dataValid(): Boolean { return data.isDataValid() }

    fun getVelocity(channel: Int): Short { return data.velocities[channel] }
}