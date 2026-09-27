package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.math.Pose

object HiveManager {
    var hiveOnRight = true

    fun getTargetPose(): Pose {
        var targetX = 62.0
        val targetY = if (hiveOnRight) 56.0 else 144.0 - 56.0
        return Pose(targetX, targetY, 0.0)
    }

    fun setHiveRight() { hiveOnRight = true }
    fun setHiveLeft() { hiveOnRight = false }
    fun flipHive() { hiveOnRight = !hiveOnRight }
}