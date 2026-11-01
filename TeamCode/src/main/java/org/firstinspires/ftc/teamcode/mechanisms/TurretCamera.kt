package org.firstinspires.ftc.teamcode.mechanisms

import android.util.Size
import dev.nextftc.hardware.RobotController.hardwareMap
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.core.RobotHardware
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class TurretCamera : Mechanism {

    companion object {
        const val VENDOR_ID_ARDUCAM = 0x0C45
        const val PRODUCT_ID_ARDUCAM_OV9281 = 0x6366
    }

    var processor: AprilTagProcessor? = null
        private set

    var visionPortal: VisionPortal? = null
        private set

    fun initCamera() {
        if (visionPortal != null) return

        CameraCompatibilityManager.getInstance().addQuirk(
            VENDOR_ID_ARDUCAM,
            PRODUCT_ID_ARDUCAM_OV9281,
            CameraCompatibilityManager.Quirk.AVOID_LIB_USB_RESET_DEVICE
        )

        val webcam = hardwareMap.get(WebcamName::class.java, RobotHardware.CAM_TURRET.deviceName)
        val newProcessor = AprilTagProcessor.Builder()
            .setLensIntrinsics(914.0, 914.0, 640.0, 400.0)
            .build()

        val newPortal = VisionPortal.Builder()
            .setCamera(webcam)
            .addProcessor(newProcessor)
            .setCameraResolution(Size(1280, 800))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        processor = newProcessor
        visionPortal = newPortal
    }

    fun closeCamera() {
        visionPortal?.close()
        visionPortal = null
        processor = null
    }
}
