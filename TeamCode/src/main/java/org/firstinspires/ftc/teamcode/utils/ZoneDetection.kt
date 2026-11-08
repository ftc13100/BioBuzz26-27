package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.math.Pose
import kotlin.math.abs
import kotlin.math.sqrt

data class Point(val x: Double, val y: Double)

class Triangle(var pointA: Point, var pointB: Point, var pointC: Point) {
    companion object {
        fun getArea(pointA: Point, pointB: Point, pointC: Point): Double {return abs((pointB.x - pointA.x) * (pointC.y - pointA.y) - (pointC.x - pointA.x) * (pointB.y - pointA.y))}
    }
    fun getArea(): Double {return abs((pointB.x - pointA.x) * (pointC.y - pointA.y) - (pointC.x - pointA.x) * (pointB.y - pointA.y))}

    fun poseInTriangle(pose: Pose): Boolean {
        val point = Point(pose.x(), pose.y())

        val s1 = getArea(pointA, pointB, point)
        val s2 = getArea(pointB, pointC, point)
        val s3 = getArea(pointC, pointA, point)
        val totalArea = getArea(pointA, pointB, pointC)

        return abs(s1 + s2 + s3 - totalArea) <= 0.001
    }

    fun expandTriangle(r: Double) {
        val center = Point(
            (pointA.x + pointB.x + pointC.x) / 3.0,
            (pointA.y + pointB.y + pointC.y) / 3.0
        )

        fun outwardNormal(p1: Point, p2: Point): Point? {
            val dx = p2.x - p1.x
            val dy = p2.y - p1.y
            val len = sqrt(dx * dx + dy * dy)
            if (len < 1e-9) return null

            var nx = -dy / len
            var ny = dx / len

            val vx = (p1.x + p2.x) / 2.0 - center.x
            val vy = (p1.y + p2.y) / 2.0 - center.y

            if (nx * vx + ny * vy < 0) {
                nx = -nx
                ny = -ny
            }
            return Point(nx, ny)
        }

        fun offsetVertex(p: Point, n1: Point, n2: Point): Point? {
            val d1 = n1.x * p.x + n1.y * p.y + r
            val d2 = n2.x * p.x + n2.y * p.y + r

            val det = n1.x * n2.y - n1.y * n2.x
            if (abs(det) < 1e-9) return null

            return Point(
                (d1 * n2.y - n1.y * d2) / det,
                (n1.x * d2 - d1 * n2.x) / det
            )
        }

        val nAB = outwardNormal(pointA, pointB) ?: return
        val nBC = outwardNormal(pointB, pointC) ?: return
        val nCA = outwardNormal(pointC, pointA) ?: return


        pointA = offsetVertex(pointA, nCA, nAB) ?: return
        pointB = offsetVertex(pointB, nAB, nBC) ?: return
        pointC = offsetVertex(pointC, nBC, nCA) ?: return
    }
}

class Trapezoid(var pointA: Point, var pointB: Point, var pointC: Point, var pointD: Point) {
    companion object {
        fun getArea(pointA: Point, pointB: Point, pointC: Point, pointD: Point): Double {
            return abs(
                (pointA.x * pointB.y - pointB.x * pointA.y) +
                        (pointB.x * pointC.y - pointC.x * pointB.y) +
                        (pointC.x * pointD.y - pointD.x * pointC.y) +
                        (pointD.x * pointA.y - pointA.x * pointD.y)
            )
        }
    }

    fun getArea(): Double {
        return getArea(pointA, pointB, pointC, pointD)
    }

    fun poseInTrapezoid(pose: Pose): Boolean {
        val point = Point(pose.x(), pose.y())

        val s1 = Triangle.getArea(pointA, pointB, point)
        val s2 = Triangle.getArea(pointB, pointC, point)
        val s3 = Triangle.getArea(pointC, pointD, point)
        val s4 = Triangle.getArea(pointD, pointA, point)
        val totalArea = getArea()

        return abs(s1 + s2 + s3 + s4 - totalArea) <= 0.001
    }

    fun expandTrapezoid(r: Double) {
        val center = Point(
            (pointA.x + pointB.x + pointC.x + pointD.x) / 4.0,
            (pointA.y + pointB.y + pointC.y + pointD.y) / 4.0
        )

        fun outwardNormal(p1: Point, p2: Point): Point? {
            val dx = p2.x - p1.x
            val dy = p2.y - p1.y
            val len = sqrt(dx * dx + dy * dy)
            if (len < 1e-9) return null

            var nx = -dy / len
            var ny = dx / len

            val vx = (p1.x + p2.x) / 2.0 - center.x
            val vy = (p1.y + p2.y) / 2.0 - center.y

            if (nx * vx + ny * vy < 0) {
                nx = -nx
                ny = -ny
            }
            return Point(nx, ny)
        }

        fun offsetVertex(p: Point, n1: Point, n2: Point): Point? {
            val d1 = n1.x * p.x + n1.y * p.y + r
            val d2 = n2.x * p.x + n2.y * p.y + r

            val det = n1.x * n2.y - n1.y * n2.x
            if (abs(det) < 1e-9) return null

            return Point(
                (d1 * n2.y - n1.y * d2) / det,
                (n1.x * d2 - d1 * n2.x) / det
            )
        }

        val nAB = outwardNormal(pointA, pointB) ?: return
        val nBC = outwardNormal(pointB, pointC) ?: return
        val nCD = outwardNormal(pointC, pointD) ?: return
        val nDA = outwardNormal(pointD, pointA) ?: return

        val newA = offsetVertex(pointA, nDA, nAB) ?: return
        val newB = offsetVertex(pointB, nAB, nBC) ?: return
        val newC = offsetVertex(pointC, nBC, nCD) ?: return
        val newD = offsetVertex(pointD, nCD, nDA) ?: return

        pointA = newA
        pointB = newB
        pointC = newC
        pointD = newD
    }
}