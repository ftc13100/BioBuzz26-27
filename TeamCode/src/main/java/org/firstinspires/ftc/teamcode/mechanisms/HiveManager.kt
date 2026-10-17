import com.pedropathing.math.Pose
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.core.PoseManager
import org.firstinspires.ftc.teamcode.core.RobotHardware
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection

class HiveManager : Mechanism {
    val HIVE_PITCH_TOLERANCE = 5
    var hiveOnRight = true

    val targetTagFilter = if (PoseManager.redAlliance) "R" else "B"

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

    fun visionUpdate() = instant {
        val detection = processor.detections.filter { detection ->
            detection is AprilTagClusterDetection && detection.metadata.name[0].equals(
                targetTagFilter
            )
        }.get(0)

        val relativeAngle = detection.ftcPose.pitch
    }

    fun setHiveRight() = instant { hiveOnRight = true }
    fun setHiveLeft() = instant { hiveOnRight = false }
    fun flipHive() = instant { hiveOnRight = !hiveOnRight }

    fun getTargetPose(): Pose {
        var targetX = 62.0
        val targetY = if (hiveOnRight) 56.0 else 144.0 - 56.0
        return Pose(targetX, targetY, 0.0)
    }
}