package org.firstinspires.ftc.teamcode

import HiveManager
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import dev.nextftc.hardware.RobotController
import dev.nextftc.robot.NextRobot
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.pedro.Constants

class BeaverRobot : NextRobot {
    val intake = Intake()
    val shooter = Shooter()
    val turretHood = TurretHood()
    val spindexer = Spindexer()
    val turret by lazy { Turret(follower) }
    val hiveManager = HiveManager()

    override val mechanisms by lazy { setOf(intake, shooter, turretHood, turret, spindexer) }

    private var _follower: Follower? = null

    val follower: Follower
        get() {
            if (_follower == null) {
                _follower = Constants.create(RobotController.hardwareMap)
            }
            return _follower!!
        }

    fun updateFollower() {
        follower.update()
    }
}
