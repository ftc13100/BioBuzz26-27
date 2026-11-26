package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.robot.Mechanism
import dev.nextftc.hardware.RobotController.hardwareMap

class Drivetrain : Mechanism {
    lateinit var gamepad: Gamepad
    val frontLeft = hardwareMap.get(DcMotor::class.java, "frontLeft")
    val frontRight = hardwareMap.get(DcMotor::class.java, "frontRight")
    val backLeft = hardwareMap.get(DcMotor::class.java, "backLeft")
    val backRight = hardwareMap.get(DcMotor::class.java, "backRight")

    fun startDrivetrain(gamepad: Gamepad) {
        this.gamepad = gamepad
    }

    fun applyDrivetrain() {
        fun format(x: Float): Double {
            return x.toDouble()
        }

        val x = format(gamepad.left_stick_x)
        val y = format(-gamepad.left_stick_y)
        val rx = format(gamepad.right_stick_x)

        frontLeft.power = (y + x + rx).coerceIn(-1.0, 1.0)
        frontRight.power = (y - x - rx).coerceIn(-1.0, 1.0)
        backLeft.power = (y - x + rx).coerceIn(-1.0, 1.0)
        backRight.power = (y + x - rx).coerceIn(-1.0, 1.0)
    }

    override fun periodic() {
        if (::gamepad.isInitialized) applyDrivetrain()
    }
}