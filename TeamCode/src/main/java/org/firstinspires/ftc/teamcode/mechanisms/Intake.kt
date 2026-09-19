package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.core.RobotHardware

class Intake : Mechanism {
    val intakeMotor = NextMotor(RobotHardware.M_INTAKE.deviceName)
    private var intakeState: IntakeState

    enum class IntakeState {
        FORWARD,
        REVERSE,
        OFF
    }

    private val power: Double
    private val forward = 1.0
    private val reverse = -1.0
    private val off = 0.0

    init {
        intakeMotor.direction = NextMotor.Direction.REVERSE
        intakeState = IntakeState.OFF
        power = off
    }

    private fun setState(intakeState: IntakeState) {
        this.intakeState = intakeState
    }

    fun setSpeed(state: IntakeState): Command {
        return instant { this.setState(state) }
    }

    override fun periodic() {
        when (intakeState) {
            IntakeState.FORWARD -> intakeMotor.throttle = forward
            IntakeState.REVERSE -> intakeMotor.throttle = reverse
            IntakeState.OFF -> intakeMotor.throttle = off
        }
    }
}