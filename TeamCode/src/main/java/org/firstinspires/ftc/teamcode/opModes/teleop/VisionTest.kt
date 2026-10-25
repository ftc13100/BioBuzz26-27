package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.ivy.commands.Commands.instant
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot
import org.firstinspires.ftc.teamcode.core.PoseManager
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.abs

private const val TELEMETRY_INTERVAL = 100

@NextTeleop(name = "Vision Test (AprilTag)")
class VisionTest(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {

    private var firstOnUpdate = true
    private var lastLoopTime = 0.0
    private var maxLoopTime = 0.0
    private var minLoopTime = Double.MAX_VALUE
    private var currentLoopTime = 0.0

    init {
        val driver = CommandGamepad(gamepad1)
        val toggleAlliance = instant { PoseManager.redAlliance = !PoseManager.redAlliance }

        driver.x.onTrue(toggleAlliance)
    }

    override fun disabledPeriodic() {
        val hm = beaverRobot.hiveManager
        if (hm.turretArducam == null) {
            hm.initVision()
        }

        val processor = hm.processor
        val portal = hm.turretArducam

        val initDetections = processor?.detections ?: emptyList()
        val matchingInitClusters = initDetections.filterIsInstance<AprilTagClusterDetection>().filter { cluster ->
            val name = cluster.metadata.name
            val isMatchingAlliance = name.isNotEmpty() && name[0].equals(hm.targetTagFilter, ignoreCase = true)
            val isRightSideUp = cluster.ftcPose?.let { abs(it.roll) < 90.0 } ?: true
            isMatchingAlliance && isRightSideUp
        }

        telemetry.addData("Camera State", portal?.cameraState ?: "NOT INITIALIZED")
        telemetry.addData("Active Alliance", if (PoseManager.redAlliance) "RED ('R')" else "BLUE ('B')")
        telemetry.addData("Target Tag Filter", hm.targetTagFilter)
        telemetry.addData("# Total Detections", initDetections.size)
        telemetry.addData("# Matching Clusters", matchingInitClusters.size)
    }

    override fun start() {
        firstOnUpdate = true
        maxLoopTime = 0.0
        minLoopTime = Double.MAX_VALUE
    }

    override fun periodic() {
        val now = System.nanoTime() / 1_000_000.0

        if (firstOnUpdate) {
            lastLoopTime = now
            firstOnUpdate = false
            return
        }

        currentLoopTime = now - lastLoopTime
        lastLoopTime = now
        if (currentLoopTime > maxLoopTime) maxLoopTime = currentLoopTime
        if (currentLoopTime < minLoopTime) minLoopTime = currentLoopTime

        val hm = beaverRobot.hiveManager
        if (hm.turretArducam == null) {
            hm.initVision()
        }

        hm.analyzeLatestFrame()
        val detections: List<AprilTagDetection> = hm.processor?.detections ?: emptyList()

        telemetry.addData("Camera State", hm.turretArducam?.cameraState ?: "NOT INITIALIZED")
        telemetry.addData("Active Alliance", if (PoseManager.redAlliance) "RED ('R')" else "BLUE ('B')")
        telemetry.addData("Target Tag Filter", hm.targetTagFilter)
        telemetry.addData(
            "Loop Time",
            "%.2f ms (Hz: %.1f)".format(currentLoopTime, if (currentLoopTime > 0) 1000.0 / currentLoopTime else 0.0)
        )
        telemetry.addData("Min / Max Loop", "%.2f ms / %.2f ms".format(minLoopTime, maxLoopTime))

        val matchingClusters = detections.filterIsInstance<AprilTagClusterDetection>().filter { cluster ->
            val name = cluster.metadata.name
            val isMatchingAlliance = name.isNotEmpty() && name[0].equals(hm.targetTagFilter, ignoreCase = true)
            val isRightSideUp = cluster.ftcPose?.let { abs(it.roll) < 90.0 } ?: true
            isMatchingAlliance && isRightSideUp
        }

        telemetry.addData("# Matching Alliance Clusters", matchingClusters.size)

        if (matchingClusters.isEmpty()) {
            telemetry.addLine("\n[No matching '${hm.targetTagFilter}' alliance clusters detected]")
        } else {
            for (cluster in matchingClusters) {
                val clusterName = cluster.metadata.name
                telemetry.addLine("\n== MATCHING CLUSTER: $clusterName ==")
                telemetry.addData("  Cluster Health", "%.0f%%".format(cluster.percentClusterFound))
                cluster.ftcPose?.let { pose ->
                    val pitch = pose.pitch
                    val roll = pose.roll
                    val isUp = pitch < hm.HIVE_PITCH_THRESHOLD
                    val isRightSideUp = abs(roll) < 90.0
                    telemetry.addData("  Hive Position", if (isUp) "UP (< 8.0 deg)" else "DOWN (>= 8.0 deg)")
                    telemetry.addData("  Orientation", if (isRightSideUp) "Right-Side Up (Roll: %.1f deg)".format(roll) else "Upside Down (Roll: %.1f deg)".format(roll))
                    telemetry.addData("  Pitch (Relative Angle)", "%.2f deg".format(pitch))
                    telemetry.addData("  XYZ Distance", "X: %.1f, Y: %.1f, Z: %.1f in".format(pose.x, pose.y, pose.z))
                    telemetry.addData("  Range / Bearing", "Range: %.1f in, Bearing: %.1f deg".format(pose.range, pose.bearing))
                }
            }
        }
    }

    override fun end() {
        beaverRobot.hiveManager.closeVision()
    }
}
