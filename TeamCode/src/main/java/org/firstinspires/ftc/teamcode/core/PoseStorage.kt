package org.firstinspires.ftc.teamcode.core

import com.pedropathing.math.Pose

object PoseStorage {
    val resetPose = Pose(144.0 - Dimensions.robotWidthCenter, Dimensions.robotLengthIntake, 180.0)
    var autonomousEndPose: Pose = resetPose

    /**
     * Mirrors a pose because Pedro 3 removed it
     */
    fun mirror(pose: Pose): Pose {
        return Pose(141.5 - pose.x(), pose.y(), Math.PI - pose.heading())
    }
}

/**
 * we can probably add the flower positions in here and then search for
   the closest flower based on position and then use a pedro command
   to automatically go there
 */