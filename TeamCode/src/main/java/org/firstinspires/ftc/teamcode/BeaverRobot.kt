package org.firstinspires.ftc.teamcode

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.robot.Mechanism
import dev.nextftc.robot.NextRobot
import dev.nextftc.robot.drive.mecanumDrive
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import java.util.Set


class BeaverRobot : NextRobot {

    private val intake = Intake()
    private val drivetrain = Drivetrain()

    fun startDrive(gamepad1: Gamepad) {
        mecanumDrive(
            drivetrain.frontLeft,
            drivetrain.frontRight,
            drivetrain.backLeft,
            drivetrain.backRight,
            gamepad1
        ).schedule()
    }

    override val mechanisms = setOf(drivetrain, intake)

}