package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.math.Pose
import com.pedropathing.math.Vector
import dev.nextftc.units.radians
import org.firstinspires.ftc.teamcode.BeaverRobot
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * shot parameters using Inverse Distance Weighting (IDW) interpolation.
 */
object BiLinearShooter {
    var projectedX = 0.0
    var projectedY = 0.0

    var useZoneProjection = true
    var zoneProjectionLookahead = 0.4

    data class ShotParameters(val velocity: Double, val angle: Double)

    private data class DataPoint(
        val x: Double,
        val y: Double,
        val velocity: Double,
        val angle: Double
    )

    private val shotData = listOf(
        DataPoint(x = 0.0, y = 45.97, velocity = 1280.0, angle = 0.15),
        DataPoint(x = 0.0, y = 21.97, velocity = 1580.0, angle = 0.35),
        DataPoint(x = 24.0, y = 21.97, velocity = 1380.0, angle = 0.3),
        DataPoint(x = 48.0, y = 21.97, velocity = 1300.0, angle = 0.2),
    )

    private const val IDW_POWER = 2.0
    private const val EPSILON = 1e-6

    /**
     * Get shot parameters based on current field position and velocity.
     */
    fun getShot(x: Double, y: Double, velocity: Vector, hiveOnRight: Boolean): ShotParameters {
        val x = if (HiveManager.hiveOnRight) x else -x

        val velocityOffset = if (velocity.magnitude() > EPSILON) {
            velocity.normalized().times(velocity.magnitude() * zoneProjectionLookahead)
        } else {
            Vector(0.0, 0.0)
        }
        
        projectedX = x + velocityOffset.elements[0]
        projectedY = y + velocityOffset.elements[1]

        var distances = shotData.map { point ->
            val dx = projectedX - point.x
            val dy = projectedY - point.y
            sqrt(dx * dx + dy * dy)
        }

        if (!hiveOnRight) {
            distances = shotData.map { point ->
                val dx = -(projectedX - point.x)
                val dy = projectedY - point.y
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
     * Update shooter, hood, and spindexer based on calculated parameters.
     */
    fun applyShot(params: ShotParameters, robot: BeaverRobot) {
        robot.turretHood.targetPosition = params.angle + robot.turretHood.manualOffset
        robot.turretHood.update().schedule()
        robot.shooter.spinAtSpeed(params.velocity + robot.shooter.manualOffset).schedule()
    }
}
