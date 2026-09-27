package org.firstinspires.ftc.teamcode.core

import com.pedropathing.math.Pose

object PoseStorage {
    val resetPose = Pose(Dimensions.robotLengthCenterBack, Dimensions.robotWidthCenter, 0.0)
    var EndPose: Pose = resetPose

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