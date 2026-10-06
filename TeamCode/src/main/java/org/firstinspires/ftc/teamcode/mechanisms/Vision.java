package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;

/**
 * Vision subsystem for managing the Limelight neural network pipeline.
 * Converts raw camera target angles into absolute field coordinates.
 */
public class Vision {

    // --- MATH & CONFIGURATION CONSTANTS (MANUAL ADJUSTMENT REQUIRED) ---
    // 1. Physical camera positioning relative to the ground
    private static final double CAMERA_HEIGHT_INCHES = 8.5; // Distance from floor to camera lens center
    private static final double TARGET_HEIGHT_INCHES = 1.5; // Radius/height of the "yellow_pollen" game piece

    // 2. Camera angles
    private static final double CAMERA_MOUNT_PITCH_DEG = 25.0; // Angle the camera tilts down (0 = looking straight forward)

    // 3. Physical camera offset relative to the robot's tracking center
    private static final double CAMERA_OFFSET_X = 6.0; // Distance forward (+) or backward (-) from robot center
    private static final double CAMERA_OFFSET_Y = 0.0; // Distance left (+) or right (-) from robot center

    // --- STUBBED NETWORKTABLES / LIMELIGHT INTERFACE ---
    // Replace these stubs with your specific Limelight wrapper or standard FTC NetworkTables implementation.
    // Examples: Limelight3A class, LLResult, or raw NetworkTables handles.
    private Object limelightDevice;

    public Vision(HardwareMap hardwareMap) {
        // Initialize your Limelight hardware map device here
        // Example: this.limelightDevice = hardwareMap.get(Limelight3A.class, "limelight");
    }

    /**
     * Represents a single detected game element with field relative coordinates.
     * Matches the .getX() and .getY() calls in your Weaver builder lambda expression.
     */
    public static class GameElement {
        private final double x;
        private final double y;

        public GameElement(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() { return x; }
        public double getY() { return y; }
    }

    /**
     * Polls the Limelight for all active neural network targets and calculates
     * their global field X and Y coordinates.
     */
    public List<GameElement> getGameElements() {
        List<GameElement> elements = new ArrayList<>();

        // TODO: Fetch the latest result frame from your Limelight API
        // Example: LLResult result = limelight.getLatestResult();
        // if (result == null || !result.isValid()) return elements;

        // Mocking a loop over the multiple neural network detections shown in your image
        // Replace with your actual loop (e.g., for (LLResultTarget target : result.getTargets()))
        int detectedTargetCount = 0;

        for (int i = 0; i < detectedTargetCount; i++) {
            // Raw horizontal and vertical offset angles from the crosshair to the target
            double tx = 0.0; // Replace with target.getTargetXDegrees();
            double ty = 0.0; // Replace with target.getTargetYDegrees();

            // --- STEP 1: CALCULATE DISTANCE TO TARGET (GROUND MATH) ---
            // Formula: distance = (h2 - h1) / tan(pitch_angle + ty)
            double angleToTargetRad = Math.toRadians(CAMERA_MOUNT_PITCH_DEG + ty);
            double heightDifference = CAMERA_HEIGHT_INCHES - TARGET_HEIGHT_INCHES;

            // Prevent division by zero if camera is parallel to ground
            if (angleToTargetRad == 0) continue;
            double distanceRobotToTargetForward = heightDifference / Math.tan(angleToTargetRad);

            // Calculate sideways offset relative to camera centerline
            double distanceRobotToTargetSideways = distanceRobotToTargetForward * Math.tan(Math.toRadians(tx));

            // --- STEP 2: ACCOUNT FOR CAMERA MOUNTING POSITION ---
            // Adjusts the target position from "relative to camera" to "relative to robot center"
            double robotRelativeX = distanceRobotToTargetForward + CAMERA_OFFSET_X;
            double robotRelativeY = distanceRobotToTargetSideways + CAMERA_OFFSET_Y;

            // --- STEP 3: CONVERT TO GLOBAL FIELD COORDINATES ---
            // Get the current robot position from the follower odometry to project the point onto the field
            // Note: Since this class initializes during init(), make sure your odometry is actively updating
            double robotGlobalX = 72.0; // TODO: Replace with: follower.getPose().getX();
            double robotGlobalY = 72.0; // TODO: Replace with: follower.getPose().getY();
            double robotHeadingRad = 0.0; // TODO: Replace with: follower.getPose().getHeading();

            // Coordinate transformation matrix using robot heading
            double fieldTargetX = robotGlobalX + (robotRelativeX * Math.cos(robotHeadingRad) - robotRelativeY * Math.sin(robotHeadingRad));
            double fieldTargetY = robotGlobalY + (robotRelativeX * Math.sin(robotHeadingRad) + robotRelativeY * Math.cos(robotHeadingRad));

            elements.add(new GameElement(fieldTargetX, fieldTargetY));
        }

        return elements;
    }
}
