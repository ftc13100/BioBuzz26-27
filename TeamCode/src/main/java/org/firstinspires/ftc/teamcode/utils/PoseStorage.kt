package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.Dimensions


object PoseStorage {
    var blueAlliance = false
    var redAlliance = false
    var motif = 0
    var resetPose = Pose(Dimensions.robotWidthCenter, Dimensions.robotLengthIntake, 180.0)
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