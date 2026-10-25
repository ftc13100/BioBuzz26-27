package org.firstinspires.ftc.teamcode.mechanisms

import android.util.Size
import com.pedropathing.ivy.Command
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
    val HIVE_PITCH_THRESHOLD = 8.0

    var lastPitch = 0.0
    var hiveOnRight = true

    val isHiveUp: Boolean
        get() = lastPitch < HIVE_PITCH_THRESHOLD

    val isHiveDown: Boolean
        get() = lastPitch >= HIVE_PITCH_THRESHOLD

    val targetTagFilter: Char
        get() = if (PoseManager.redAlliance) 'R' else 'B'

    var processor: AprilTagProcessor? = null
        private set

    var turretArducam: VisionPortal? = null
        private set

    fun initVision() {
        closeVision()

        val newProcessor = AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setLensIntrinsics(914.0, 914.0, 640.0, 400.0)
            .build()

        val newPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "turretArducam"))
            .addProcessor(newProcessor)
            .setCameraResolution(Size(1280, 800))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        processor = newProcessor
        turretArducam = newPortal
    }

    fun closeVision() {
        turretArducam?.close()
        turretArducam = null
        processor = null
    }

    fun analyzeLatestFrame(): AprilTagClusterDetection? {
        val activeProcessor = processor ?: return null
        val detections = activeProcessor.detections ?: emptyList()
        val clusterDetection = detections.filterIsInstance<AprilTagClusterDetection>()
            .firstOrNull { cluster ->
                val name = cluster.metadata.name
                val isMatchingAlliance = name.isNotEmpty() && name[0].equals(targetTagFilter, ignoreCase = true)
                val isRightSideUp = cluster.ftcPose?.let { abs(it.roll) < 90.0 } ?: true
                isMatchingAlliance && isRightSideUp
            }

        if (clusterDetection?.ftcPose != null) {
            lastPitch = clusterDetection.ftcPose.pitch
        }

        return clusterDetection
    }

    fun takeSnapshot(pauseAfter: Boolean = false): AprilTagClusterDetection? {
        val result = analyzeLatestFrame()
        if (pauseAfter) {
            turretArducam?.stopStreaming()
        }
        return result
    }

    fun visionUpdate(): Command = instant {
        analyzeLatestFrame()
    }

    fun setHiveRight(): Command = instant { hiveOnRight = true }
    fun setHiveLeft(): Command = instant { hiveOnRight = false }
    fun flipHive(): Command = instant { hiveOnRight = !hiveOnRight }

    fun getTargetPose(): Pose {
        val targetX = 62.0
        val targetY = if (hiveOnRight) 56.0 else 144.0 - 56.0
        return Pose(targetX, targetY, 0.0)
    }
}
