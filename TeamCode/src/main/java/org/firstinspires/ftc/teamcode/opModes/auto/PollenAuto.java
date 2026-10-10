package org.firstinspires.ftc.teamcode.opModes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.teamcode.mechanisms.Vision;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "PollenAuto", group = "Autonomous")
public class PollenAuto extends OpMode {
    private Follower follower;
    private Vision vision;
    private PathResult result;
    private boolean pathGenerated = false;

    @Override
    public void init() {
        // 1. Initialize the follower FIRST so it isn't null when passed to Vision
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(24, 118, 0));

        // 2. Assign to the class-level field (no duplicate type prefix)
        vision = new Vision(hardwareMap, follower);

        // 3. Configure Weaver
        Weaver.setConfig(new PathConfig()
                .intakeWidth(8.0)
                .robotWidth(16.0)
                .robotHeight(16.0));

        telemetry.addData("Status", "Initialized. Waiting for vision targets...");
    }

    @Override
    public void init_loop() {
        // Update the follower to ensure localizer tracking stays healthy
        follower.update();

        // Grab the latest game elements detected by the Limelight
        java.util.List<Vision.GameElement> elements = vision.getGameElements();
        telemetry.addData("Detected Elements", elements.size());

        // Only build the path once elements are found and we haven't generated a path yet
        if (!elements.isEmpty() && !pathGenerated) {
            result = Weaver.builder()
                    .start(new Point(24, 118, 0))
                    .addObstacle(new CircleZone(new Point(48, 90), 4))
                    .targets(elements, e -> new Point(e.getX(), e.getY()))
                    .maxTargets(4)
                    .build();

            // Mark as true to satisfy the IDE warning and stop rebuilding every cycle
            pathGenerated = true;

            telemetry.addData("Path Status", "Path Generated successfully!");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        // Run the generated path if targets were found during initialization
        if (result != null) {
            follower.follow(PedroPathingConverter.toPath(result));
        } else {
            telemetry.addData("Warning", "No targets detected before match start!");
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        // Keep updating Pedro Pathing control loops during execution
        follower.update();
    }
}
