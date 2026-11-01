package org.firstinspires.ftc.teamcode.opModes.teleop

import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import org.firstinspires.ftc.teamcode.BeaverRobot
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection

@NextTeleop(name = "Vision Stream Test")
class VisionStreamTest(beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {

    private val camera = beaverRobot.camera

    init {
        camera.initCamera()
    }

    override fun disabledPeriodic() {
        val portal = camera.visionPortal
        val processor = camera.processor

        telemetry.addData("Camera State", portal?.cameraState ?: "NOT INITIALIZED")
        telemetry.addData("FPS", "%.1f".format(portal?.fps ?: 0.0f))

        val detections: List<AprilTagDetection> = processor?.detections ?: emptyList()
        telemetry.addData("Total Detections", detections.size)
    }

    override fun periodic() {
        val portal = camera.visionPortal
        val processor = camera.processor

        telemetry.addData("Camera State", portal?.cameraState ?: "NOT INITIALIZED")
        telemetry.addData("FPS", "%.1f".format(portal?.fps ?: 0.0f))

        val detections: List<AprilTagDetection> = processor?.detections ?: emptyList()
        telemetry.addData("Total Detections", detections.size)

        for (detection in detections) {
            when (detection) {
                is AprilTagSingleDetection -> {
                    telemetry.addLine("\n== SINGLE TAG ID: ${detection.id} ==")
                }
                is AprilTagClusterDetection -> {
                    telemetry.addLine("\n== CLUSTER: ${detection.metadata?.name ?: "Unknown"} ==")
                    telemetry.addData("  Cluster Health", "%d%%", detection.percentClusterFound)
                }
            }
            detection.ftcPose?.let { pose ->
                val isUp = pose.pitch < 8.0
                telemetry.addData("  Hive Status", if (isUp) "UP (< 8.0 deg)" else "DOWN (>= 8.0 deg)")
                telemetry.addData("  Pitch", "%.2f deg".format(pose.pitch))
                telemetry.addData("  XYZ (in)", "X: %.1f, Y: %.1f, Z: %.1f".format(pose.x, pose.y, pose.z))
                telemetry.addData("  Range / Bearing", "Range: %.1f in, Bearing: %.1f deg".format(pose.range, pose.bearing))
            }
        }
    }

    override fun end() {
        camera.closeCamera()
    }
}
