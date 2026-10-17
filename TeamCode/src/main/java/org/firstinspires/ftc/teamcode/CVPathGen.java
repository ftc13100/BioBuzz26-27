package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
//15,8.5
//16.6,50
//+2.2,-11.5
//30.5
public class CVPathGen {
    private Limelight3A ll;
    private Follower follower;
    private boolean isRed;
    CVCostAlgo algo = new CVCostAlgo();

    //Limelight constants in (in and deg
    Telemetry telemetry;

    public ArrayList<Pose> simBall = new ArrayList<>();
    public boolean simulation = false;
    public Pose simBotPose = new Pose(0,0,0);
    private static Matrix cameraAdjustments(Pose robotPose) {
        //accounts for cam heading pitch and stuff
        double heading = robotPose.heading();
        //- pi / 2 to make 0 facing right instead of up

        Matrix m = Matrix.rotationTransform(heading - Math.PI / 2).times(CVConstants.createRotationX(-CVConstants.CAMERA_PITCH));
        m = m.times(CVConstants.xyzConverter);
        return m;
    }

    public CVPathGen(HardwareMap hardwareMap, Follower follower, boolean isRed, Telemetry telemetry){
        this.follower = follower;
        this.isRed = isRed;
        this.telemetry = telemetry;


        ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.start();
        ll.pipelineSwitch(7);
    }
    public CVPathGen(boolean isRed){

        this.isRed = isRed;



    }
    public ArrayList<Pose> getBalls(boolean sim){//get balls from limelight
        if(sim) return simBall;
        ArrayList<double[]> relativeBalls = new ArrayList<>();
        ArrayList<Pose> absoluteBalls = new ArrayList<>();
        LLResult result = ll.getLatestResult();

        if (result != null && result.isValid()) {//get deg of each ball from ll
            Pose po = follower.pose();
            for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
                if (!"yellow_pollen".equalsIgnoreCase(dr.getClassName())) { // changed to pollen only to test
                    continue;
                }
                relativeBalls.add(new double[]{Math.toRadians(dr.getTargetXDegrees()), Math.toRadians(dr.getTargetYDegrees())});
            }
            for(double[] ball : relativeBalls){
                absoluteBalls.add(ballConverter(ball, po));
            }
        }
        drawAll(absoluteBalls);
        return absoluteBalls;
    }
    public Pose ballConverter(double[] ball, Pose curpose){
        Matrix cameraRay = new Matrix(
                new double[][] {
                        {Math.tan(ball[0])},
                        {Math.tan(ball[1])},
                        {1.0}
                }
        );
        //convert cam ray to absolute ray so like ray in field view not relative to bot
        Matrix absoluteRay = cameraAdjustments(curpose).times(cameraRay);

        //find forward distance from cam to ball
        double dist = -CVConstants.CAMERA_HEIGHT / absoluteRay.get(2, 0);
       // telemetry.addData("dist", dist);

        Matrix cam = new Matrix(// absolute cam pose
                new double[][] {
                        {curpose.x() + CVConstants.FORWARD_OFFSET * Math.cos(curpose.heading())},
                        {curpose.y() + CVConstants.FORWARD_OFFSET * Math.sin(curpose.heading())},
                        {CVConstants.CAMERA_HEIGHT}
                }
        );
        //add absolute cam pose and vector of cam to ball
        double x = cam.get(0,0) + absoluteRay.get(0,0) * dist;
        double y = cam.get(1,0) + absoluteRay.get(1,0) * dist;
        return new Pose(x, y, 0);
    }

    private ArrayList<Pose> sortBalls() {
        ArrayList<Pose> balls = getBalls(simulation);
        Pose po = simulation ? simBotPose : follower.pose();
        //filter in only balls on our side
        balls.removeIf(ball -> isRed ? ball.x() > 70 : ball.x() < 70);
        // sorts based on distance to balls
        ArrayList<Pose> best = new ArrayList<>();
        best.addAll(algo.costFunc(balls, po));
        return algo.combiner(best, po);
    }


    public Path pathGen() {
        ArrayList<Pose> ballPoses = new ArrayList<>();
        ballPoses.add(follower.pose());
        ballPoses.addAll(sortBalls());
        if (ballPoses.size()<2) {
            return null;
        }
        //so for some reason u cant put a list inside and u need an array of poses
        Pose[] balls = ballPoses.toArray(new Pose[0]);
        drawChosen(ballPoses);
        Path p = Paths.through(balls).tangent();
        return p;

    }
    //simulation
    public ArrayList<Pose> pathGenSim() {
        ArrayList<Pose> ballPoses = new ArrayList<>();
        ballPoses.add(simBotPose);
        ballPoses.addAll(sortBalls());
        if (ballPoses.size()<2) {
            return null;
        }
        //so for some reason u cant put a list inside and u need an array of poses
        Pose[] balls = ballPoses.toArray(new Pose[0]);
        //drawChosen(ballPoses);
        Path p = Paths.through(balls).tangent();
        return ballPoses;
    }

    //drawing code does NOT work rip
    public void drawChosen(ArrayList<Pose> balls){
        for (int i = 0; i < balls.size(); i++) {
            Pose ball = balls.get(i);
//draw bot as black circle and put green ring around chosen balls
            if (i == 0) {
            } else {
            }

        }
       // FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
    public void drawAll(ArrayList<Pose> balls){
//draw all balls yellow
        for (int i = 0; i < balls.size(); i++) {
            Pose ball = balls.get(i);
        }
    }
}
