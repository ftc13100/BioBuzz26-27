package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import dev.nextftc.robot.drive.mecanumDrive

class Drivetrain : Mechanism {
    val frontLeft = NextMotor("frontLeft")
    val frontRight = NextMotor("frontRight")
    val backLeft = NextMotor("backLeft")
    val backRight = NextMotor("backRight")

    fun startDrive(gamepad: Gamepad) {
        mecanumDrive(frontLeft, frontRight, backLeft, backRight, gamepad).schedule()
    }
}