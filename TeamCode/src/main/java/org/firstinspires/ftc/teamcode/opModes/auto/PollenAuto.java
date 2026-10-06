package org.firstinspires.ftc.teamcode.opModes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.teamcode.mechanisms.Vision;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class PollenAuto extends OpMode {
    private Follower follower;
    private Vision vision;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));

        Weaver.setConfig(new PathConfig()
                .intakeWidth(8.0)
                .robotWidth(16.0)
                .robotHeight(16.0));

        PathResult result = Weaver.builder()
                .start(new Point(72, 72, 0))
                .addObstacle(new CircleZone(new Point(90, 60), 6))
                .targets(vision.getGameElements(), e -> new Point(e.getX(), e.getY()))
                .maxTargets(4)
                .build();

        follower.follow(PedroPathingConverter.toPath(result));
    }

    @Override
    public void loop() {
        follower.update();
    }
}