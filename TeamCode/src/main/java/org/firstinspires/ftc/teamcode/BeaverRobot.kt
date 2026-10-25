package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import dev.nextftc.hardware.RobotController
import dev.nextftc.robot.NextRobot
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.pedro.Constants

class BeaverRobot : NextRobot {
    val hiveManager by lazy { HiveManager() }
    val intake by lazy { Intake() }
    val shooter by lazy { Shooter() }
    val turretHood by lazy { TurretHood() }
    val turret by lazy { Turret(follower, hiveManager) }
    val spindexer by lazy { Spindexer() }

    override val mechanisms by lazy { setOf(hiveManager, intake, shooter, turretHood, turret, spindexer) }

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