package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.PanelsConfigurables
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.parallel
import dev.nextftc.robot.opmode.NextOpMode
import dev.nextftc.robot.opmode.NextTeleop
import dev.nextftc.robot.triggers.CommandGamepad
import org.firstinspires.ftc.teamcode.BeaverRobot
import org.firstinspires.ftc.teamcode.core.OctoquadPorts
import org.firstinspires.ftc.teamcode.mechanisms.Shooter
import kotlin.math.abs

@Configurable
@NextTeleop(name = "Shooter Test & Tune")
class ShooterTuner(val beaverRobot: BeaverRobot) : NextOpMode(beaverRobot) {
    private val panelsTelemetry = PanelsTelemetry.telemetry

    companion object {
        @JvmField var targetVel = 1600.0
    }

    private var firstOnUpdate = true
    private var lastLoopTime = 0.0
    private var maxLoopTime = 0.0
    private var minLoopTime = Double.MAX_VALUE

    override fun start() {
        PanelsConfigurables.refreshClass(this::class)
        PanelsConfigurables.refreshClass(Shooter::class)

        val driver = CommandGamepad(gamepad1)

        driver.a.onTrue(beaverRobot.shooter.spinAtSpeed(targetVel))
        driver.b.onTrue(beaverRobot.shooter.stop())
        driver.dpadUp.onTrue(beaverRobot.gate.close())
        driver.dpadDown.onTrue(beaverRobot.gate.open())

        // Toggle Voltage Compensation on/off
        driver.y.onTrue(instant { Shooter.useVoltageComp = !Shooter.useVoltageComp })

        // Toggle Intake
        driver.rightBumper.toggleOnTrue(beaverRobot.intake.intake())

        // Toggle Shoot Feeder (Intake + Transfer)
        driver.x.toggleOnTrue(
            parallel(
                beaverRobot.intake.intake(),
                beaverRobot.transfer.shoot()
            )
        )
    }

    override fun periodic() {
        beaverRobot.octoquadReader.readAll()
        val now = System.nanoTime() / 1_000_000.0
        if (firstOnUpdate) {
            lastLoopTime = now
            firstOnUpdate = false
            return
        }

        val loopTime = now - lastLoopTime
        lastLoopTime = now
        if (loopTime > maxLoopTime) maxLoopTime = loopTime
        if (loopTime < minLoopTime) minLoopTime = loopTime

        if (beaverRobot.shooter.active) {
            Shooter.targetVelocity = targetVel
        }

        val currentVelocity = abs(beaverRobot.octoquadReader.getVelocity(OctoquadPorts.Q_SHOOTER.channel).toDouble())
        val power = beaverRobot.shooter.motor.throttle
        val error = targetVel - currentVelocity

        telemetry.addData("Voltage Comp Enabled", Shooter.useVoltageComp)
        telemetry.addData("Loop Time", "%.2f ms".format(loopTime))
        telemetry.addData("Min Loop Time", "%.2f ms".format(minLoopTime))
        telemetry.addData("Max Loop Time", "%.2f ms".format(maxLoopTime))
        telemetry.update()

        panelsTelemetry.addData("targetVelocity", targetVel)
        panelsTelemetry.addData("currentVelocity", currentVelocity)
        panelsTelemetry.addData("error", error)
        panelsTelemetry.addData("power", power)
        panelsTelemetry.addData("voltageCompEnabled", Shooter.useVoltageComp)
        panelsTelemetry.addData("loopTime", loopTime)
        panelsTelemetry.update(telemetry)
    }
}
