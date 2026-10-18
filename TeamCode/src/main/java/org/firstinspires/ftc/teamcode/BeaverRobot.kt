package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.mechanisms.HiveManager
import com.pedropathing.follower.Follower
import dev.nextftc.hardware.RobotController
import dev.nextftc.robot.NextRobot
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.pedro.Constants

class BeaverRobot : NextRobot {
    val hiveManager by lazy { HiveManager() }
    val intake = Intake()
    val shooter = Shooter()
    val turretHood = TurretHood()
    val turret by lazy { Turret(follower, hiveManager) }

    override val mechanisms by lazy { setOf(hiveManager, intake, shooter, turretHood, turret) }

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
