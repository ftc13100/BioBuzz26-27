package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.math.Pose
import kotlin.math.abs
import kotlin.math.sqrt

object ZoneDetection {
    private val radius = 10.0

    val closeShootingZone = Triangle(
        Point(0.0, 144.0),
        Point(144.0, 144.0),
        Point(72.0, 72.0)
    )

    val farShootingZone = Triangle(
        Point(42.25, 0.0),
        Point(72.0, 29.75),
        Point(101.75, 0.0)
    )

    val scaledCloseShootingZone = expandTriangle(closeShootingZone, radius)
    val scaledFarShootingZone = expandTriangle(farShootingZone, radius)

    fun signedArea(a: Point, b: Point, c: Point) = abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y))

    fun poseInTriangle(pose: Pose, triangle: Triangle): Boolean {
        val point = Point(pose.x(), pose.y())

        val s1 = signedArea(triangle.pointA, triangle.pointB, point)
        val s2 = signedArea(triangle.pointB, triangle.pointC, point)
        val s3 = signedArea(triangle.pointC, triangle.pointA, point)
        val totalArea = signedArea(triangle.pointA, triangle.pointB, triangle.pointC)

        return abs(s1 + s2 + s3 - totalArea) <= 0.001
    }

    fun expandTriangle(triangle: Triangle, r: Double): Triangle {
        fun getOutwardNormal(p1: Point, p2: Point, pCenter: Point): Point {
            val dx = p2.x - p1.x
            val dy = p2.y - p1.y
            val len = sqrt(dx * dx + dy * dy)

            var nx = -dy / len
            var ny = dx / len

            val mx = (p1.x + p2.x) / 2.0
            val my = (p1.y + p2.y) / 2.0

            val vx = mx - pCenter.x
            val vy = my - pCenter.y

            if (nx * vx + ny * vy < 0) {
                nx = -nx
                ny = -ny
            }
            return Point(nx, ny)
        }

        val center = Point(
            (triangle.pointA.x + triangle.pointB.x + triangle.pointC.x) / 3.0,
            (triangle.pointA.y + triangle.pointB.y + triangle.pointC.y) / 3.0
        )

        val nAB = getOutwardNormal(triangle.pointA, triangle.pointB, center)
        val nBC = getOutwardNormal(triangle.pointB, triangle.pointC, center)
        val nCA = getOutwardNormal(triangle.pointC, triangle.pointA, center)

        fun offsetVertex(p: Point, n1: Point, n2: Point): Point {
            val d1 = n1.x * (p.x + r * n1.x) + n1.y * (p.y + r * n1.y)
            val d2 = n2.x * (p.x + r * n2.x) + n2.y * (p.y + r * n2.y)

            val det = n1.x * n2.y - n1.y * n2.x
            val newX = (d1 * n2.y - n1.y * d2) / det
            val newY = (n1.x * d2 - d1 * n2.x) / det

            return Point(newX, newY)
        }

        val newA = offsetVertex(triangle.pointA, nCA, nAB)
        val newB = offsetVertex(triangle.pointB, nAB, nBC)
        val newC = offsetVertex(triangle.pointC, nBC, nCA)

        return Triangle(newA, newB, newC)
    }
}

data class Triangle(val pointA: Point, val pointB: Point, val pointC: Point)
data class Point(val x: Double, val y: Double)
