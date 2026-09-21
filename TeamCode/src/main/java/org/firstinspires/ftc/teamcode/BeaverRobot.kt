package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import dev.nextftc.hardware.RobotController
import dev.nextftc.robot.NextRobot
import org.firstinspires.ftc.teamcode.mechanisms.*
import org.firstinspires.ftc.teamcode.pedro.Constants

class BeaverRobot : NextRobot {
    private var _follower: Follower? = null
    
    val follower: Follower
        get() {
            if (_follower == null) {
                _follower = Constants.create(RobotController.hardwareMap)
            }
            return _follower!!
        }

    val intake = Intake()
    val shooter = Shooter()
    val shooterAngle = ShooterAngle()
    val turret by lazy { NewTurret(follower) }
    val spindexer = Spindexer()
    
    override val mechanisms = setOf(intake, shooter, shooterAngle, turret, spindexer)

    fun updateFollower() {
        follower.update()
    }
}
