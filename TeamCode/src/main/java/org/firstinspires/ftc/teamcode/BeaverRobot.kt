package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import dev.nextftc.hardware.RobotController
import dev.nextftc.robot.NextRobot
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.pedro.Constants

class BeaverRobot : NextRobot {
    val octoquadReader by lazy { OctoquadReader() }
    val drivetrain by lazy { Drivetrain() }
    val hiveManager by lazy { HiveManager() }
    val intake by lazy { Intake() }
    val transfer by lazy { Transfer() }
    val shooter by lazy { Shooter(octoquadReader) }
    val turret by lazy { Turret(follower, hiveManager) }
    val turretHood by lazy { TurretHood() }
    val gate by lazy { Gate() }
    val ramp by lazy { Ramp() }

    override val mechanisms by lazy { setOf(drivetrain, hiveManager, intake, transfer, shooter, turret, turretHood, gate, ramp, octoquadReader) }

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