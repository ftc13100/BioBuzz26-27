package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class Vision {

    private static final double CAMERA_HEIGHT_INCHES = 10.75;
    // 1.4 inches represents the center of the 2.8" ball where the bounding box targets
    private static final double TARGET_HEIGHT_INCHES = 1.4;
    private static final double CAMERA_MOUNT_PITCH_DEG = 0.0;
    private static final double CAMERA_OFFSET_X = 5.0;
    private static final double CAMERA_OFFSET_Y = 0.0;

    private final Limelight3A limelight;
    private final Follower follower;

    public Vision(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7); // Your active neural detector pipeline
        limelight.start();
    }

    public static class GameElement {
        private final double x;
        private final double y;

        public GameElement(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }
    }

    public List<GameElement> getGameElements() {
        List<GameElement> elements = new ArrayList<>();
        LLResult result = limelight.getLatestResult();

        // If the result is null or stale, return empty right away
        if (result == null || !result.isValid()) {
            return elements;
        }

        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

        // If detector array is empty, we fall back to the main crosshair target metrics
        if (detectorResults == null || detectorResults.isEmpty()) {
            // Check if the camera sees any general target in its crosshair
            if (result.getTa() > 0) {
                processTarget(result.getTx(), result.getTy(), elements);
            }
            return elements;
        }

        for (LLResultTypes.DetectorResult target : detectorResults) {
            String className = target.getClassName();
            double confidence = target.getConfidence();

            // Lowered confidence check to 0.30 to perfectly match your network model's lower bound
            if (!"yellow_pollen".equals(className) || confidence < 0.30) {
                continue;
            }

            processTarget(target.getTargetXDegrees(), target.getTargetYDegrees(), elements);
        }

        return elements;
    }

    private void processTarget(double tx, double ty, List<GameElement> elements) {
        // Since the camera is parallel to the ground (0 deg pitch) and looking DOWN,
        // ty is negative. Subtracting ty flips the sign to positive for correct trigonometry.
        double angleToTargetRad = Math.toRadians(
                CAMERA_MOUNT_PITCH_DEG - ty
        );

        double heightDifference = CAMERA_HEIGHT_INCHES - TARGET_HEIGHT_INCHES;

        if (Math.abs(Math.tan(angleToTargetRad)) < 1e-6) {
            return;
        }

        // This yields a correct, positive forward distance
        double distanceRobotToTargetForward = heightDifference / Math.tan(angleToTargetRad);

        if (distanceRobotToTargetForward <= 0) {
            return;
        }

        double distanceRobotToTargetSideways = distanceRobotToTargetForward * Math.tan(Math.toRadians(tx));

        double robotRelativeX = distanceRobotToTargetForward + CAMERA_OFFSET_X;
        double robotRelativeY = distanceRobotToTargetSideways + CAMERA_OFFSET_Y;

        // Integrates with Pedro Pathing's actual odometry pose safely
        double robotGlobalX = follower.pose().x();
        double robotGlobalY = follower.pose().y();
        double robotHeadingRad = follower.pose().heading();

        double fieldTargetX = robotGlobalX
                + robotRelativeX * Math.cos(robotHeadingRad)
                - robotRelativeY * Math.sin(robotHeadingRad);

        double fieldTargetY = robotGlobalY
                + robotRelativeX * Math.sin(robotHeadingRad)
                + robotRelativeY * Math.cos(robotHeadingRad);

        elements.add(new GameElement(fieldTargetX, fieldTargetY));
    }
}
