package org.firstinspires.ftc.teamcode.mechanisms

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import dev.nextftc.hardware.RobotController
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.hardware.sensors.NextAnalogInput
import dev.nextftc.robot.Mechanism
import dev.nextftc.units.radians
import org.firstinspires.ftc.teamcode.core.RobotHardware
import org.firstinspires.ftc.teamcode.utils.PoseStorage
import kotlin.math.abs

class Spindexer : Mechanism {
    val motor = NextMotor(RobotHardware.M_SPINDEXER.deviceName)
    val analogS = NextAnalogInput({ RobotController.hardwareMap.get(RobotHardware.A_SPINDEXER.deviceName) as com.qualcomm.robotcore.hardware.AnalogInput })
    
    val color0: NormalizedColorSensor by lazy { RobotController.hardwareMap.get(NormalizedColorSensor::class.java, RobotHardware.I2C_COLOR0.deviceName) }
    val color1: NormalizedColorSensor by lazy { RobotController.hardwareMap.get(NormalizedColorSensor::class.java, RobotHardware.I2C_COLOR1.deviceName) }
    val color2: NormalizedColorSensor by lazy { RobotController.hardwareMap.get(NormalizedColorSensor::class.java, RobotHardware.I2C_COLOR2.deviceName) }

    @JvmField var target = 0.0
    @JvmField var kP = -0.0015
    @JvmField var kD = -0.000033

    enum class State { PID, MANUAL }
    var state = State.MANUAL

    enum class SpindexerColor { PURPLE, GREEN, EMPTY }
    var cached0 = SpindexerColor.EMPTY
    var cached1 = SpindexerColor.EMPTY
    var cached2 = SpindexerColor.EMPTY

    val ENCODER_MAX = 4000.0
    val STEP = ENCODER_MAX / 3.0
    val INTAKE_ABS_POS = 339.0
    val ABS_ENC_V_MAX = 3.225

    var initDone = false
    var intakePos1 = 0.0
    var intakePos2 = 0.0
    var intakePos3 = 0.0
    var targetPosition = 0.0
    var targetReached = false
    var atIntakePos = false
    
    @JvmField var spinShotSpeed = 0.9

    init {
        motor.zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE
    }

    override fun periodic() {
        motor.positionConstants.apply {
            this.kP = this@Spindexer.kP
            this.kD = this@Spindexer.kD
        }

        if (!initDone) {
            val absEncP = (analogS.rawVoltage.magnitude / ABS_ENC_V_MAX) * ENCODER_MAX
            var digEncOffset = (digEncLimitV() - absEncP) % ENCODER_MAX
            if (digEncOffset < 0.0) digEncOffset += ENCODER_MAX

            var p1 = (INTAKE_ABS_POS + digEncOffset) % ENCODER_MAX
            if (p1 >= STEP) {
                p1 = (p1 + STEP) % ENCODER_MAX
                if (p1 >= STEP) p1 = (p1 + STEP) % ENCODER_MAX
            }
            intakePos1 = p1
            intakePos2 = intakePos1 + STEP
            intakePos3 = intakePos2 + STEP
            initDone = true
        }
        
        if (state == State.PID) {
            motor.update()
        }
    }

    fun digEncLimitV(): Double {
        var enc = motor.encoderPosition.magnitude % ENCODER_MAX
        if (enc < 0.0) enc += ENCODER_MAX
        return enc
    }

    fun intakePos(adj: Double = 0.0): Double {
        val curPos = (digEncLimitV() + adj) % ENCODER_MAX
        var movePos = 0.0

        if (curPos <= intakePos1) movePos = intakePos1 - curPos
        else if (curPos <= intakePos2) movePos = intakePos2 - curPos
        else if (curPos <= intakePos3) movePos = intakePos3 - curPos
        else movePos = intakePos1 + ENCODER_MAX - curPos

        if (movePos > (STEP - 40.0)) movePos -= STEP
        
        atIntakePos = true
        return motor.encoderPosition.magnitude + movePos
    }

    fun spinShot(): Command = instant {
        state = State.MANUAL
        motor.throttle = spinShotSpeed
        atIntakePos = false
    }.requiring(this)

    fun spinShotIndex(): Command = instant {
        state = State.MANUAL
        motor.throttle = 0.5
        atIntakePos = false
    }.requiring(this)

    fun shootTimedOrPosition(): Command {
        var startTime = 0L
        var startPos = 0.0
        return Command.build()
            .setStart {
                state = State.MANUAL
                startTime = System.currentTimeMillis()
                startPos = motor.encoderPosition.magnitude
                motor.throttle = spinShotSpeed
                atIntakePos = false
            }
            .setDone {
                val now = System.currentTimeMillis()
                val timeDone = (now - startTime) >= 500
                val positionDone = abs(motor.encoderPosition.magnitude - startPos) >= ENCODER_MAX
                timeDone || positionDone
            }
            .setEnd { _ -> motor.throttle = 0.0 }
            .requiring(this)
    }

    fun stop(): Command = instant {
        state = State.MANUAL
        motor.throttle = 0.0
        clearColors()
    }.requiring(this)

    fun toIntakePos(): Command = Command.build()
        .setStart {
            state = State.PID
            targetReached = false
            targetPosition = intakePos()
            motor.setPositionSetpoint(targetPosition.radians)
        }
        .setDone { abs(motor.encoderPosition.magnitude - targetPosition) < 40.0 }
        .setEnd { _ -> state = State.MANUAL }
        .requiring(this)

    fun autoIndex(b3: Int): Command = instant {
        val b0 = colorToDigit(detectColorRGB(color0))
        val b1 = colorToDigit(detectColorRGB(color1))
        val b2 = colorToDigit(detectColorRGB(color2))
        val dexIndex = b0 * 81 + b1 * 27 + b2 * 9 + b3 * 3 + PoseStorage.motif
        val targetIdx = dexing[dexIndex]
        
        when (targetIdx) {
            0 -> indexTo(0.0).schedule()
            1 -> indexTo(STEP).schedule()
            2 -> indexTo(STEP * 2).schedule()
        }
    }.requiring(this)

    fun spin() {
        state = State.PID
        motor.setPositionSetpoint(target.radians)
    }

    private fun indexTo(offset: Double): Command = Command.build()
        .setStart {
            state = State.PID
            targetPosition = intakePos() + offset
            motor.setPositionSetpoint(targetPosition.radians)
        }
        .setDone { abs(motor.encoderPosition.magnitude - targetPosition) < 40.0 }
        .setEnd { _ -> state = State.MANUAL }
        .requiring(this)

    fun detectColorRGB(sensor: NormalizedColorSensor): SpindexerColor {
        val colors = sensor.normalizedColors
        if (colors.alpha < 0.15) return SpindexerColor.EMPTY
        return when {
            colors.green > colors.red && colors.green > colors.blue -> SpindexerColor.GREEN
            colors.blue > colors.red && colors.blue > colors.green -> SpindexerColor.PURPLE
            else -> SpindexerColor.EMPTY
        }
    }

    private fun colorToDigit(color: SpindexerColor): Int = when (color) {
        SpindexerColor.EMPTY -> 0
        SpindexerColor.GREEN -> 2
        SpindexerColor.PURPLE -> 1
    }

    fun clearColors() {
        cached0 = SpindexerColor.EMPTY
        cached1 = SpindexerColor.EMPTY
        cached2 = SpindexerColor.EMPTY
    }

    val isBusy: Boolean
        get() = state == State.PID || (state == State.MANUAL && abs(motor.throttle) > 0.1)

    val isFull: Boolean
        get() = (detectColorRGB(color0) != SpindexerColor.EMPTY &&
                 detectColorRGB(color1) != SpindexerColor.EMPTY &&
                 detectColorRGB(color2) != SpindexerColor.EMPTY)

    fun getArtifactCount(): Int {
        var count = 0
        if (detectColorRGB(color0) != SpindexerColor.EMPTY) count++
        if (detectColorRGB(color1) != SpindexerColor.EMPTY) count++
        if (detectColorRGB(color2) != SpindexerColor.EMPTY) count++
        return count
    }

    private val dexing = intArrayOf(-1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 2, 0, 1,
        1, 2, 0, 0, 1, 2, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0, 2, 0, 1, 1, 2, 0, 0,
        1, 2, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0,
        0, 0, 1, 0, 0, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0,
        0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 1, 2, 0, 0, 1, 2, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1,
        2, 0, 0, 1, 2, 2, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 2, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0,
        1, 1, 2, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0, 0, 1, 2, 2, 0, 1, 1, 2, 0,
        0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0)
}
