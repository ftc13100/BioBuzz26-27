package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "Single Servo Tester")
class SingleServoTest : LinearOpMode() {
    override fun runOpMode() {
        val servoNames = listOf("ramp", "turret", "gate", "hood")
        var selectedIndex = 0

        var lastUp = false
        var lastDown = false

        while (!isStarted && !isStopRequested) {
            val up = gamepad1.dpad_up
            val down = gamepad1.dpad_down

            if (up && !lastUp) {
                selectedIndex = (selectedIndex - 1).coerceAtLeast(0)
            }

            if (down && !lastDown) {
                selectedIndex = (selectedIndex + 1).coerceAtMost(servoNames.lastIndex)
            }

            lastUp = up
            lastDown = down

            telemetry.addData("Selected", servoNames[selectedIndex])
            telemetry.addLine("")
            telemetry.addLine("DPAD UP/DOWN: Select servo")
            telemetry.addLine("Press START to continue")
            telemetry.update()

            idle()
        }

        if (isStopRequested) return

        val servo = hardwareMap.get(Servo::class.java, servoNames[selectedIndex])

        var position = 0.5
        var step = 0.05

        var lastA = false
        var lastB = false
        var lastLB = false
        var lastRB = false

        servo.position = position

        waitForStart()

        while (opModeIsActive()) {
            val a = gamepad1.a
            val b = gamepad1.b
            val lb = gamepad1.left_bumper
            val rb = gamepad1.right_bumper

            if (a && !lastA) {
                position = (position + step).coerceAtMost(1.0)
                servo.position = position
            }

            if (b && !lastB) {
                position = (position - step).coerceAtLeast(0.0)
                servo.position = position
            }

            if (lb && !lastLB) {
                step = (step - 0.01).coerceAtLeast(0.01)
            }

            if (rb && !lastRB) {
                step = (step + 0.01).coerceAtMost(0.5)
            }

            lastA = a
            lastB = b
            lastLB = lb
            lastRB = rb

            telemetry.addData("Servo", servoNames[selectedIndex])
            telemetry.addData("Position", position)
            telemetry.addData("Step", step)

            telemetry.addLine("Controls:")
            telemetry.addLine("  A/B - Increase/Decrease Position")
            telemetry.addLine("  RB/LB - Increase/Decrease Step")

            telemetry.update()
        }
    }
}