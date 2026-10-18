package org.firstinspires.ftc.teamcode.opModes.teleop

import com.pedropathing.math.Pose
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import org.firstinspires.ftc.teamcode.BeaverRobot

@NextTeleop("Hive Manager Test")
class HiveManagerTest(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {
    override fun periodic() {
        beaverRobot.hiveManager.visionUpdate(Pose(72.0, 20.0, 90.0))

        telemetry.addData("HiveOnRight", beaverRobot.hiveManager.hiveOnRight)
    }

    override fun end() {
        beaverRobot.hiveManager.turretArducam.close()
    }
}