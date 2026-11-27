package org.firstinspires.ftc.teamcode.core

object Dimensions {
    // Field
    val fieldLength = 141.5

    // Robot
    val robotWidth = 13.66
    val robotLength = 15.27
    val robotWidthCenter = robotWidth / 2
    val robotLengthCenterBack = 7.59
    val robotLengthCenterIntake = robotLength - robotLengthCenterBack

    // Flower
    val flowerLength = 5.9
    val distanceFlowerEdgeFieldRightCorner = 44.25
    val distanceFlowerEdgeFieldLeftCorner = fieldLength - flowerLength - distanceFlowerEdgeFieldRightCorner
}