package org.firstinspires.ftc.teamcode.mechanisms

import android.util.Size
import com.pedropathing.ivy.Command
import com.pedropathing.math.Pose
import dev.nextftc.hardware.RobotController.hardwareMap
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.core.PoseManager
import org.firstinspires.ftc.teamcode.core.RobotHardware
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

    private var exposureConfigured = false

    companion object {
        const val VENDOR_ID_ARDUCAM = 0x0C45
        const val PRODUCT_ID_ARDUCAM_OV9281 = 0x6366
    }

    fun initVision() {
        closeVision()
        exposureConfigured = false

        CameraCompatibilityManager.getInstance().addQuirk(
            VENDOR_ID_ARDUCAM,
            PRODUCT_ID_ARDUCAM_OV9281,
            CameraCompatibilityManager.Quirk.AVOID_LIB_USB_RESET_DEVICE
        )

        val newProcessor = AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setLensIntrinsics(914.0, 914.0, 640.0, 400.0)
            .build()

        val newPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, RobotHardware.CAM_TURRET.deviceName))
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
        exposureConfigured = false
    }

    private fun configureExposureIfReady() {
        if (exposureConfigured) return
        val portal = turretArducam ?: return
        val state = portal.cameraState
        if (state == VisionPortal.CameraState.STREAMING || state == VisionPortal.CameraState.CAMERA_DEVICE_READY) {
            try {
                val exposureControl = portal.getCameraControl(ExposureControl::class.java)
                if (exposureControl != null && exposureControl.isModeSupported(ExposureControl.Mode.ContinuousAuto)) {
                    exposureControl.mode = ExposureControl.Mode.ContinuousAuto
                    exposureConfigured = true
                }
            } catch (_: Exception) {
            }
        }
    }

    fun analyzeLatestFrame(): AprilTagClusterDetection? {
        configureExposureIfReady()
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
