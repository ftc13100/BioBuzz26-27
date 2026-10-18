package org.firstinspires.ftc.teamcode.opModes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.PoseManager
import org.firstinspires.ftc.teamcode.mechanisms.HiveManager
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection

@TeleOp(name = "Vision Test (AprilTag)", group = "Vision")
class VisionTest : LinearOpMode() {

    override fun runOpMode() {
        telemetry.addLine("Init Vision")
        telemetry.update()

        val hiveManager = HiveManager()

        telemetry.addData("Target Tag Filter", hiveManager.targetTagFilter)
        telemetry.addData("Red Alliance", PoseManager.redAlliance)
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            val detections: List<AprilTagDetection> = hiveManager.processor.detections ?: emptyList()

            telemetry.addData("# Detections Found", detections.size)
            telemetry.addData("Hive Side", if (hiveManager.hiveOnRight) "RIGHT" else "LEFT")
            telemetry.addData("Target Tag Filter", hiveManager.targetTagFilter)

            val matchingClusterDetections = detections.filter { detection ->
                detection is AprilTagClusterDetection &&
                        detection.metadata?.name?.isNotEmpty() == true &&
                        detection.metadata.name[0].toString().equals(hiveManager.targetTagFilter, ignoreCase = true)
            }

            telemetry.addData("Matching Clusters Found", matchingClusterDetections.size)

            for (detection in detections) {
                if (detection is AprilTagClusterDetection) {
                    val clusterName = detection.metadata?.name ?: "Unknown Cluster"
                    telemetry.addLine("\n== Cluster Tag: $clusterName ==")
                    if (detection.ftcPose != null) {
                        telemetry.addData("  Pitch (Relative Angle)", "%.2f deg".format(detection.ftcPose.pitch))
                        telemetry.addData("  XYZ", "X: %.1f, Y: %.1f, Z: %.1f".format(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z))
                    }
                } else if (detection is AprilTagSingleDetection) {
                    val tagName = detection.metadata?.name ?: "Single Tag"
                    telemetry.addLine("\n== Single Tag ID ${detection.id} ($tagName) ==")
                    if (detection.ftcPose != null) {
                        telemetry.addData("  XYZ", "X: %.1f, Y: %.1f, Z: %.1f".format(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z))
                    }
                }
            }

            telemetry.update()
        }

        hiveManager.turretArducam.close()
    }
}
