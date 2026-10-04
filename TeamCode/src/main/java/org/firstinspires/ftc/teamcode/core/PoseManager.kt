package org.firstinspires.ftc.teamcode.core

import com.pedropathing.math.Pose
import kotlin.math.sqrt

object PoseManager {
    val d = Dimensions
    val masterResetPose = Pose(d.robotLengthCenterBack, d.robotWidthCenter, 0.0)
    val resetPoses = arrayOf(
        masterResetPose,
        Pose(d.robotLengthCenterBack, d.lengthFlowerTipFieldCorner - d.robotWidthCenter, 0.0),
        Pose(d.robotLengthCenterBack, d.lengthFlowerTipFieldCorner + d.flowerLength + d.robotWidthCenter, 0.0),
    )
    var EndPose: Pose = masterResetPose

    fun getResetPose(robotPose: Pose): Pose {
        var closestPose = resetPoses[0]
        var closestPoseDist = getDistance(robotPose, closestPose)

        for (pose in resetPoses) {
            val dist = getDistance(robotPose, pose)
            if (dist < closestPoseDist) {closestPose = pose; closestPoseDist = dist}
        }

        return closestPose
    }

    fun getDistance(p1: Pose, p2: Pose): Double {
        val dx = p1.x() - p2.x()
        val dy = p1.y() - p2.y()
        return sqrt(dx * dx + dy * dy)
    }

    fun mirror(pose: Pose): Pose {
        return Pose(141.5 - pose.x(), pose.y(), Math.PI - pose.heading())
    }
}