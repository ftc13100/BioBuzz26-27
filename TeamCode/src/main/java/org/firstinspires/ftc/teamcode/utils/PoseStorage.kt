package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.math.Pose


object PoseStorage {
    var blueAlliance = false
    var redAlliance = false
    var autonomousEndPose: Pose = Pose(0.0, 0.0, 0.0)
}


/* we can probably add the flower positions in here and then search for
   the closest flower based on position and then use a pedro command
   to automatically go there
 */