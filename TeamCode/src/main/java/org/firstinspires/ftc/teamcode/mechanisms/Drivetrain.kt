package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.hardware.RobotController
import dev.nextftc.hardware.RobotController.controlHub
import dev.nextftc.hardware.RobotController.expansionHub
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import dev.nextftc.robot.drive.mecanumDrive
import org.firstinspires.ftc.teamcode.core.RobotHardware

class Drivetrain : Mechanism {

    val frontLeft = NextMotor(RobotHardware.M_WHEEL_FL.deviceName)
    val frontRight = NextMotor(RobotHardware.M_WHEEL_FR.deviceName)
    val backLeft = NextMotor(RobotHardware.M_WHEEL_BL.deviceName)
    val backRight = NextMotor(RobotHardware.M_WHEEL_BR.deviceName)


    init {
        //frontRight.direction = NextMotor.Direction.REVERSE
        //backRight.direction = NextMotor.Direction.REVERSE
    }

    fun startDrive(gamepad: Gamepad) {
        mecanumDrive(frontLeft, frontRight, backLeft, backRight, gamepad).schedule()
    }

}