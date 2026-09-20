package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import dev.nextftc.hardware.RobotController
import dev.nextftc.robot.NextRobot
import org.firstinspires.ftc.teamcode.mechanisms.Intake
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
    
    override val mechanisms = setOf(intake)

    fun updateFollower() {
        follower.update()
    }
}
