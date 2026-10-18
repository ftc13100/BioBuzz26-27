package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.math.Pose
import dev.nextftc.hardware.RobotController.hardwareMap
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.core.PoseManager
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs

class HiveManager : Mechanism {
    val HIVE_PITCH_TOLERANCE = 15
    var hiveOnRight = true

    val targetTagFilter: Char = if (PoseManager.redAlliance) 'R' else 'B'

    val processor = AprilTagProcessor.Builder()
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .build()

    val turretArducam = VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName::class.java, "turretArducam"))
        .addProcessor(processor)
        .build()

    fun visionUpdate(robotPose: Pose) = instant {
        val detections = processor.detections.filter { detection ->
            detection is AprilTagClusterDetection && detection.metadata.name[0] == targetTagFilter
        }

        if (detections.size == 0) {
            setHiveOpposite(robotPose)
        } else if (abs(detections.get(0).ftcPose.pitch) > HIVE_PITCH_TOLERANCE) {
            setHiveOpposite(robotPose)
        }
    }

    fun setHiveRight() = instant { hiveOnRight = true }
    fun setHiveLeft() = instant { hiveOnRight = false }
    fun flipHive() = instant { hiveOnRight = !hiveOnRight }
    fun setHiveOpposite(robotPose: Pose) = instant {
        if (robotPose.y() < 72) setHiveRight()
        else setHiveLeft()
    }

    fun getTargetPose(): Pose {
        var targetX = 62.0
        val targetY = if (hiveOnRight) 56.0 else 144.0 - 56.0
        return Pose(targetX, targetY, 0.0)
    }
}