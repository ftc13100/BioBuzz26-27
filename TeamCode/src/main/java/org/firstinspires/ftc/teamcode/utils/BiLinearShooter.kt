package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.math.Pose
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.BeaverRobot
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * shot parameters using Inverse Distance Weighting (IDW) interpolation.
 */
object BiLinearShooter {
    val goalClose = Pose(4.0, 139.5, 0.0)
    val goalFar = Pose(3.0, 139.0, 0.0)
    var projectedX = 0.0
    var projectedY = 0.0

    var useZoneProjection = true
    var zoneProjectionLookahead = 0.4

    data class ShotParameters(val velocity: Double, val angle: Double, val spinspeed: Double)

    private data class DataPoint(
        val x: Double,
        val y: Double,
        val velocity: Double,
        val angle: Double,
        val spinspeed: Double
    )

    private val shotData = listOf(
        DataPoint(24.0, 117.97, 1820.0, 0.800, 0.55),
        DataPoint(48.0, 117.97, 1620.0, 0.600, 0.80),
        DataPoint(96.0, 117.97, 1280.0, 0.150, 1.00),
        DataPoint(72.0, 117.97, 1460.0, 0.400, 1.00),
        DataPoint(72.0, 93.97, 1520.0, 0.350, 1.00),
        DataPoint(24.0, 93.97, 1860.0, 0.700, 0.50),
        DataPoint(96.0, 93.97, 1380.0, 0.300, 1.00),
        DataPoint(48.0, 93.97, 1760.0, 0.700, 0.60),
        DataPoint(120.0, 93.97, 1300.0, 0.200, 1.00),
        DataPoint(72.0, 69.97, 1700.0, 0.650, 0.85),
        DataPoint(96.0, 69.97, 1520.0, 0.550, 0.75),
        DataPoint(48.0, 69.97, 1740.0, 0.600, 0.60),
        DataPoint(72.0, 45.97, 1740.0, 0.600, 0.55),
        DataPoint(98.03, 24.00, 1880.0, 0.650, 0.70),
        DataPoint(72.0, 21.97, 2100.0, 0.800, 0.70),
        DataPoint(50.03, 21.97, 2060.0, 0.550, 0.70),
        DataPoint(89.0, 8.70, 1980.0, 0.550, 0.65),
        DataPoint(54.4, 6.1, 1980.0, 0.750, 0.60)
    )

    private const val IDW_POWER = 2.0
    private const val EPSILON = 1e-6

    /**
     * Get shot parameters based on current field position and velocity.
     */
    fun getShot(x: Double, y: Double, velocity: Vector): ShotParameters {
        // Apply lookahead based on robot velocity
        val velocityOffset = if (velocity.magnitude() > EPSILON) {
            velocity.normalized().times(velocity.magnitude() * zoneProjectionLookahead)
        } else {
            Vector(0.0, 0.0)
        }
        
        projectedX = x + velocityOffset.elements[0]
        projectedY = y + velocityOffset.elements[1]

        val distances = shotData.map { point ->
            val dx = projectedX - point.x
            val dy = projectedY - point.y
            sqrt(dx * dx + dy * dy)
        }

        val minDistance = distances.minOrNull() ?: 0.0
        if (minDistance < EPSILON) {
            val exactPoint = shotData[distances.indexOf(minDistance)]
            return ShotParameters(exactPoint.velocity, exactPoint.angle, exactPoint.spinspeed)
        }

        val weights = distances.map { 1.0 / it.pow(IDW_POWER) }
        val totalWeight = weights.sum()

        val velocityInterp = shotData.zip(weights).sumOf { it.first.velocity * it.second } / totalWeight
        val angleInterp = shotData.zip(weights).sumOf { it.first.angle * it.second } / totalWeight
        val spinInterp = shotData.zip(weights).sumOf { it.first.spinspeed * it.second } / totalWeight

        return ShotParameters(velocityInterp, angleInterp, spinInterp)
    }

    /**
     * Update shooter, hood, and spindexer based on calculated parameters.
     */
    fun applyShot(params: ShotParameters, robot: BeaverRobot) {
        robot.shooterAngle.targetPosition = params.angle + robot.shooterAngle.manualOffset
        robot.shooterAngle.update().schedule()
        robot.shooter.spinAtSpeed(params.velocity + robot.shooter.manualOffset).schedule()
        robot.spindexer.spinShotSpeed = params.spinspeed
    }
}
