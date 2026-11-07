package org.firstinspires.ftc.teamcode.opModes.teleop

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad
import dev.nextftc.hardware.actuators.NextServo
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import org.firstinspires.ftc.teamcode.BeaverRobot

@NextTeleop(name = "OctoQuad & Turret Test", group = "Tuners")
class OctoQuadTurretTest(beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {

    private val octoquad: OctoQuad? by lazy {
        try {
            hardwareMap[OctoQuad::class.java, "octoquad"]
        } catch (_: Exception) {
            hardwareMap.getAll(OctoQuad::class.java).firstOrNull()
        }
    }

    private val turretServo: NextServo? by lazy {
        try {
            NextServo("turret")
        } catch (_: Exception) {
            null
        }
    }

    private var servoPosition = 0.0

    private var firstLoop = true
    private var lastLoopTimeMs = 0.0
    private var maxLoopTimeMs = 0.0
    private var minLoopTimeMs = Double.MAX_VALUE

    override fun start() {
        servoPosition = 0.0
        try {
            turretServo?.position = servoPosition
        } catch (_: Exception) {}
        firstLoop = true
        maxLoopTimeMs = 0.0
        minLoopTimeMs = Double.MAX_VALUE
    }

    override fun periodic() {
        val nowMs = System.nanoTime() / 1_000_000.0

        if (firstLoop) {
            lastLoopTimeMs = nowMs
            firstLoop = false
        } else {
            val loopTimeMs = nowMs - lastLoopTimeMs
            lastLoopTimeMs = nowMs
            if (loopTimeMs > maxLoopTimeMs) maxLoopTimeMs = loopTimeMs
            if (loopTimeMs < minLoopTimeMs) minLoopTimeMs = loopTimeMs

            telemetry.addData(
                "Loop Time",
                "%.2f ms (Min: %.2f, Max: %.2f)".format(
                    loopTimeMs,
                    if (minLoopTimeMs == Double.MAX_VALUE) 0.0 else minLoopTimeMs,
                    maxLoopTimeMs
                )
            )
        }

        if (gamepad1.a) {
            servoPosition = 0.0
            try { turretServo?.position = servoPosition } catch (_: Exception) {}
        } else if (gamepad1.y) {
            servoPosition = 1.0
            try { turretServo?.position = servoPosition } catch (_: Exception) {}
        } else if (gamepad1.left_bumper) {
            servoPosition = (servoPosition - 0.01).coerceIn(0.0, 1.0)
            try { turretServo?.position = servoPosition } catch (_: Exception) {}
        } else if (gamepad1.right_bumper) {
            servoPosition = (servoPosition + 0.01).coerceIn(0.0, 1.0)
            try { turretServo?.position = servoPosition } catch (_: Exception) {}
        }

        val oq = octoquad
        if (oq != null) {
            val quadPosPort2 = oq.readSinglePosition_Caching(2)
            val quadVelPort2 = oq.readSingleVelocity_Caching(2)
            telemetry.addData("OctoQuad Port 2 Position (Ticks)", quadPosPort2)
            telemetry.addData("OctoQuad Port 2 Velocity (Counts/sec)", quadVelPort2)
        } else {
            telemetry.addData("OctoQuad", "Not found")
        }

        telemetry.addData("Turret Servo Target Pos", "%.2f".format(servoPosition))
        val ts = turretServo
        if (ts != null) {
            try {
                telemetry.addData("Turret Servo Actual Pos", "%.2f".format(ts.position))
            } catch (_: Exception) {
                telemetry.addData("Turret Servo", "Error reading position")
            }
        } else {
            telemetry.addData("Turret Servo", "Not found")
        }

        telemetry.update()
    }
}
