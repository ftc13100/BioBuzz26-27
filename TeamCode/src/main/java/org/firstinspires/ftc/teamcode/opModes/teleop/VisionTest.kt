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
        val hiveManager = HiveManager()

        // Live Preview during INIT phase
        while (!isStarted && !isStopRequested) {
            val initDetections = hiveManager.processor.detections ?: emptyList()
            telemetry.addData("Target Tag Filter", hiveManager.targetTagFilter)
            telemetry.addData("Red Alliance", PoseManager.redAlliance)
            telemetry.addData("# Init Detections", initDetections.size)
            telemetry.update()
            sleep(50)
        }

        waitForStart()

        var firstOnUpdate = true
        var lastLoopTime = 0.0
        var maxLoopTime = 0.0
        var minLoopTime = Double.MAX_VALUE
        var currentLoopTime = 0.0

        while (opModeIsActive()) {
            val now = System.nanoTime() / 1_000_000.0

            if (firstOnUpdate) {
                lastLoopTime = now
                firstOnUpdate = false
            } else {
                currentLoopTime = now - lastLoopTime
                lastLoopTime = now
                if (currentLoopTime > maxLoopTime) maxLoopTime = currentLoopTime
                if (currentLoopTime < minLoopTime) minLoopTime = currentLoopTime
            }

            val detections: List<AprilTagDetection> = hiveManager.processor.detections ?: emptyList()

            telemetry.addData("Loop Time", "%.2f ms (Min: %.2f, Max: %.2f)".format(currentLoopTime, if (minLoopTime == Double.MAX_VALUE) 0.0 else minLoopTime, maxLoopTime))
            telemetry.addData("Loop Hz", "%.2f".format(if (currentLoopTime > 0) 1000.0 / currentLoopTime else 0.0))
            telemetry.addData("# Detections Found", detections.size)
            telemetry.addData("Hive Side", if (hiveManager.hiveOnRight) "RIGHT" else "LEFT")
            telemetry.addData("Target Tag Filter", hiveManager.targetTagFilter)

            val matchingClusterDetections = detections.filterIsInstance<AprilTagClusterDetection>().filter { cluster ->
                val name = cluster.metadata?.name
                !name.isNullOrEmpty() && name[0].equals(hiveManager.targetTagFilter, ignoreCase = true)
            }

            telemetry.addData("Matching Clusters Found", matchingClusterDetections.size)

            for (detection in detections) {
                if (detection is AprilTagClusterDetection) {
                    val clusterName = detection.metadata?.name ?: "Unknown Cluster"
                    telemetry.addLine("\n== Cluster Tag: $clusterName ==")
                    detection.ftcPose?.let { pose ->
                        telemetry.addData("  Pitch (Relative Angle)", "%.2f deg".format(pose.pitch))
                        telemetry.addData("  XYZ", "X: %.1f, Y: %.1f, Z: %.1f".format(pose.x, pose.y, pose.z))
                    }
                } else if (detection is AprilTagSingleDetection) {
                    val tagName = detection.metadata?.name ?: "Single Tag"
                    telemetry.addLine("\n== Single Tag ID ${detection.id} ($tagName) ==")
                    detection.ftcPose?.let { pose ->
                        telemetry.addData("  XYZ", "X: %.1f, Y: %.1f, Z: %.1f".format(pose.x, pose.y, pose.z))
                    }
                }
            }

            telemetry.update()
        }

        try {
            hiveManager.turretArducam.close()
        } catch (_: Exception) {}
    }
}
