package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.follower.ManualDrive
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot
import org.firstinspires.ftc.teamcode.utils.PoseStorage


@NextTeleop
class MainTeleop(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {

    override fun start() {
        beaverRobot.follower.setPose(PoseStorage.autonomousEndPose)

        // Bind controls in start()
        val driver = CommandGamepad(gamepad1)
        driver.rightBumper.whileTrue(beaverRobot.intake.intake())
        driver.leftBumper.whileTrue(beaverRobot.intake.outtake())
    }

    override fun periodic() {
        ManualDrive.driveOrHold(
            beaverRobot.follower,
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
        )
        
        beaverRobot.updateFollower()

        telemetry.addData("Robot Position", beaverRobot.follower.pose())
    }
}
