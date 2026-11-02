package org.firstinspires.ftc.teamcode.opModes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.core.RobotHardware

@TeleOp(name = "Distance Sensor Test")
class DistanceSensorTest : LinearOpMode() {
    override fun runOpMode() {
        val distanceIntake = hardwareMap.get(DigitalChannel::class.java, RobotHardware.DISTANCE_INTAKE.deviceName).apply {
            mode = DigitalChannel.Mode.INPUT
        }

        val loopTimer = ElapsedTime()

        waitForStart()

        loopTimer.reset()

        while (opModeIsActive()) {
            val loopTimeMs = loopTimer.milliseconds()
            loopTimer.reset()

            val isBroken = distanceIntake.state

            telemetry.addData("Beam State", if (isBroken) "DETECTED" else "NOTHING")
            telemetry.addData("Loop (ms)", "%.2f ms", loopTimeMs)
            telemetry.update()
        }
    }
}