package org.firstinspires.ftc.teamcode;



import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.CVPathGen;

import java.util.ArrayList;


@TeleOp
public class CVTest extends LinearOpMode {
    private CVPathGen pather;
    private Follower follower;
    public static boolean isRed = true;
    ArrayList<Pose> allBalls = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(24,118,Math.toRadians(-90)));
        follower.update();
        pather = new CVPathGen(hardwareMap,follower, isRed, telemetry);

        waitForStart();
        while (opModeIsActive()){
            follower.update();
            allBalls = pather.getBalls(false);
            if(gamepad1.aWasPressed()){
                Path path = pather.pathGen();
                if(path != null){
                    follower.follow(path);
                }
            }
            if(follower.isBusy() && follower.atParametricEnd()){
                follower.stop(); //fixes final wierd turns
            }
            for (Pose ball: allBalls) {
                telemetry.addLine("(" + ball.x() + ", "+ ball.y() + ")");
            }
            telemetry.addData("Robot X", follower.pose().x());
            telemetry.addData("Robot Y", follower.pose().y());
            telemetry.addData("Remaining", follower.remainingDistance());
            telemetry.addData("Dist to endpoint", follower.distanceToEndpoint());
            telemetry.addData("Completion", follower.completion());
            telemetry.update();
        }
    }
}
