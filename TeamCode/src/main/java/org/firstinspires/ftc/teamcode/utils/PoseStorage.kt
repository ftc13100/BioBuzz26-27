package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.math.Pose


object PoseStorage {
    var blueAlliance = false
    var redAlliance = false
    var motif = 0
    var autonomousEndPose: Pose = Pose(0.0, 0.0, 0.0)

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