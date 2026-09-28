package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.teamcode.BeaverRobot
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Shot parameters using interpolation.
 */
object BiLinearShooter {

    var projectedX = 0.0
    var projectedY = 0.0

    data class ShotParameters(val velocity: Double, val angle: Double)

    private data class DataPoint(
        val x: Double,
        val y: Double,
        val velocity: Double,
        val angle: Double
    )

    private val shotData = listOf(
        DataPoint(72.0, 72.0, 900.0, 0.800),
        )

    private const val IDW_POWER = 2.0
    private const val EPSILON = 1e-6

    /**
     * Get shot parameters based directly on current position.
     */
    fun getShot(x: Double, y: Double, hiveOnRight: Boolean): ShotParameters {
        val posX = if (hiveOnRight) x else -x
        val posY = y

        projectedX = posX
        projectedY = posY

        var distances = shotData.map { point ->
            val dx = posX - point.x
            val dy = posY - point.y
            sqrt(dx * dx + dy * dy)
        }

        if (!hiveOnRight) {
            distances = shotData.map { point ->
                val dx = -(posX - point.x)
                val dy = posY - point.y
                sqrt(dx * dx + dy * dy)
            }
        }

        val minDistance = distances.minOrNull() ?: 0.0
        if (minDistance < EPSILON) {
            val exactPoint = shotData[distances.indexOf(minDistance)]
            return ShotParameters(exactPoint.velocity, exactPoint.angle)
        }

        val weights = distances.map { 1.0 / it.pow(IDW_POWER) }
        val totalWeight = weights.sum()

        val velocityInterp = shotData.zip(weights).sumOf { it.first.velocity * it.second } / totalWeight
        val angleInterp = shotData.zip(weights).sumOf { it.first.angle * it.second } / totalWeight

        return ShotParameters(velocityInterp, angleInterp)
    }

    /**
     * Update shooter and hood based on calculated parameters.
     */
    fun applyShot(params: ShotParameters, robot: BeaverRobot) {
        robot.turretHood.targetPosition = params.angle + robot.turretHood.manualOffset
        robot.turretHood.update().schedule()
        robot.shooter.spinAtSpeed(params.velocity + robot.shooter.manualOffset).schedule()
    }
}
