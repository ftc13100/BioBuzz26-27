package org.firstinspires.ftc.teamcode.core

import com.pedropathing.math.Pose
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

object PoseManager {
    val center = Pose(0.0, 0.0, 0.0)
    val d = Dimensions
    val masterResetPose = Pose(d.robotLengthCenterBack, d.robotWidthCenter, 0.0)
    var EndPose: Pose = masterResetPose

    val baseResetPostions = arrayOf(
        // Right Corner
        Pose(d.robotLengthCenterBack, d.robotWidthCenter, 0.0),
        Pose(d.robotWidthCenter, d.robotLengthCenterBack, 90.0),
        // Right Side Flower
        Pose(d.robotLengthCenterBack, d.distanceFlowerEdgeFieldRightCorner - d.robotWidthCenter, 0.0),
        Pose(d.robotWidthCenter, d.distanceFlowerEdgeFieldRightCorner - d.robotLengthCenterBack, 90.0),
        // Left Side Flower
        Pose(d.robotLengthCenterBack, d.distanceFlowerEdgeFieldRightCorner + d.flowerLength + d.robotWidthCenter, 0.0),
        Pose(d.robotWidthCenter, d.distanceFlowerEdgeFieldRightCorner + d.flowerLength + d.robotLengthCenterBack, 90.0),
    )

    var resetPoses = ArrayList<Pose>()

    init {
        var rotation = 0.0
        for (i in 1..4) {
            baseResetPostions.forEach { resetPoses.add(rotatePose(it, center, rotation)) }
            rotation += Math.PI / 2
        }
    }

    fun getResetPose(robotPose: Pose): Pose {
        var closestPose = resetPoses[0]
        var closestPoseDist = getPoseDistance(robotPose, closestPose)

        for (pose in resetPoses) {
            val dist = getPoseDistance(robotPose, pose)
            if (dist < closestPoseDist) {closestPose = pose; closestPoseDist = dist}
        }

        return closestPose
    }

    fun getPoseDistance(p1: Pose, p2: Pose): Double {
        val dx = p1.x() - p2.x()
        val dy = p1.y() - p2.y()
        if (abs(p1.heading() - p2.heading()) > 10) return sqrt(dx * dx + dy * dy) * 2
        else return sqrt(dx * dx + dy * dy)
    }

    fun rotatePose(target: Pose, center: Pose, angleRadians: Double): Pose {
        val dx = target.x() - center.x()
        val dy = target.y() - center.y()

        val cos = cos(angleRadians)
        val sin = sin(angleRadians)

        val rotatedDx = dx * cos - dy * sin
        val rotatedDy = dx * sin + dy * cos

        val newX = center.x() + rotatedDx
        val newY = center.y() + rotatedDy
        val newHeading = target.heading() + angleRadians

        return Pose(newX, newY, newHeading)
    }

    fun mirrorPose(pose: Pose): Pose {
        return Pose(141.5 - pose.x(), pose.y(), Math.PI - pose.heading())
    }
}