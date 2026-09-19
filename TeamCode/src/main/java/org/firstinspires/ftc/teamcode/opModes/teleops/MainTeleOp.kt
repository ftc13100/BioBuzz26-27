package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import dev.nextftc.robot.triggers.Trigger.Companion.defaultEventLoop
import org.firstinspires.ftc.teamcode.BeaverRobot


@NextTeleop
class MainTeleop(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {
    override fun start() {
        beaverRobot.startDrive(gamepad1)
    }
}
