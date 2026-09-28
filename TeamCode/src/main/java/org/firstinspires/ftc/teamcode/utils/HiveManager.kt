package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.Dimensions

object HiveManager {

    @JvmField var hiveOnRight = true

    @JvmField var lineYRight = 51.5
    @JvmField var lineXMin = 49.0
    @JvmField var lineXMax = 69.0
    @JvmField var midXRobot = 59.0

    /**
     * Calculates the dynamic target Pose on the hive opening line based on current robot Y position
     * Uses 3 point linear interpolation that we can tune
     */
    fun getTargetPose(robotX: Double): Pose {
        val minRobotX = Dimensions.robotWidthCenter
        val maxRobotX = Dimensions.FIELD_WIDTH - Dimensions.robotWidthCenter
        val midTargetX = (lineXMin + lineXMax) / 2.0

        val targetX = when {
            robotX <= minRobotX -> lineXMax
            robotX <= midXRobot -> {
                val t = (robotX - minRobotX) / (midXRobot - minRobotX)
                lineXMax + t * (midTargetX - lineXMax)
            }
            robotX < maxRobotX -> {
                val t = (robotX - midXRobot) / (maxRobotX - midXRobot)
                midTargetX + t * (lineXMin - midTargetX)
            }
            else -> lineXMin
        }

        val targetY = if (hiveOnRight) lineYRight else (Dimensions.FIELD_WIDTH - lineYRight)
        return Pose(targetX, targetY, 0.0)
    }

    /**
     * no arg defaults to using center point
     */
    fun getTargetPose(): Pose = getTargetPose(midXRobot)

    fun setHiveRight() { hiveOnRight = true }
    fun setHiveLeft() { hiveOnRight = false }
    fun flipHive() { hiveOnRight = !hiveOnRight }
}
