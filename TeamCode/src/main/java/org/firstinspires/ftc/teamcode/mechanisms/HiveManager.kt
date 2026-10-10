import com.pedropathing.math.Pose
import dev.nextftc.hardware.webcams.NextLimelight
import dev.nextftc.robot.Mechanism

class HiveManager : Mechanism {
    val turretLimelight = NextLimelight("limelight")
    var hiveOnRight = true

    fun startPolling() = instant { turretLimelight.startReading(pipeline = 0, hz = 100) }

    fun getTargetPose(): Pose {
        var targetX = 62.0
        val targetY = if (hiveOnRight) 56.0 else 144.0 - 56.0
        return Pose(targetX, targetY, 0.0)
    }

    fun setHiveRight() = instant { hiveOnRight = true }
    fun setHiveLeft() = instant { hiveOnRight = false }
    fun flipHive() = instant { hiveOnRight = !hiveOnRight }
}