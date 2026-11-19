package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad
import dev.nextftc.hardware.RobotController.hardwareMap
import dev.nextftc.robot.Mechanism

class OctoquadReader : Mechanism {
    val octoquad = hardwareMap.get(OctoQuad::class.java, "octoquad")

    fun readAll() {
        val s = "unfinsihed"
    }
}