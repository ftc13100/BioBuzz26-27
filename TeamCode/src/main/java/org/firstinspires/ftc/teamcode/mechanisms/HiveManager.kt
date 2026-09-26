package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import com.pedropathing.math.Pose
import dev.nextftc.robot.Mechanism

class HiveManager : Mechanism {
    var hiveOnRight: Boolean
    init {
        hiveOnRight = true
    }

    fun getTargetPose(robotPose: Pose): Pose {
        var targetX = (0.5 * robotPose.x()) + 40
        val targetY = if (hiveOnRight) 56.0 else 144.0 - 56.0
        return Pose(targetX, targetY, 0.0)
    }

    fun flipHive(): Command = instant { hiveOnRight = !hiveOnRight }
}