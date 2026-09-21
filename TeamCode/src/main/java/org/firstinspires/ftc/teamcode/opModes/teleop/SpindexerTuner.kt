package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.ivy.commands.Commands.instant
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot


@NextTeleop(name = "Spindexer Test & Tune", group = "Tuners")
class SpindexerTuner(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {

    override fun start() {
        val driver = CommandGamepad(gamepad1)
        val operator = CommandGamepad(gamepad2)

        // Manual target adjustment
        driver.dpadUp.onTrue(instant { beaverRobot.spindexer.target += 100.0 })
        driver.dpadDown.onTrue(instant { beaverRobot.spindexer.target -= 100.0 })
        driver.dpadRight.onTrue(instant { beaverRobot.spindexer.target += 10.0 })
        driver.dpadLeft.onTrue(instant { beaverRobot.spindexer.target -= 10.0 })

        // Auto-indexing tests
        operator.x.onTrue(beaverRobot.spindexer.autoIndex(0))
        operator.y.onTrue(beaverRobot.spindexer.autoIndex(1))
        operator.b.onTrue(beaverRobot.spindexer.autoIndex(2))

        operator.a.toggleOnTrue(beaverRobot.spindexer.spinShot())
    }

    override fun periodic() {

        if (beaverRobot.spindexer.state == org.firstinspires.ftc.teamcode.mechanisms.Spindexer.State.PID) {
            beaverRobot.spindexer.spin()
        }

        val pos = beaverRobot.spindexer.motor.encoderPosition.magnitude
        val current = beaverRobot.spindexer.motor.current.magnitude

        telemetry.addData("State", beaverRobot.spindexer.state)
        telemetry.addData("Position", "%.1f".format(pos))
        telemetry.addData("Target", beaverRobot.spindexer.target)
        telemetry.addData("Current (Amps)", "%.3f".format(current))

        telemetry.addData("S0", beaverRobot.spindexer.detectColorRGB(beaverRobot.spindexer.color0))
        telemetry.addData("S1", beaverRobot.spindexer.detectColorRGB(beaverRobot.spindexer.color1))
        telemetry.addData("S2", beaverRobot.spindexer.detectColorRGB(beaverRobot.spindexer.color2))

        telemetry.update()
    }
}
