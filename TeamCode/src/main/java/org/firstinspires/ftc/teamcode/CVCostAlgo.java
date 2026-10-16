package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class CVCostAlgo {
    private ArrayList<ArrayList<Pose>> permutationsGen(ArrayList<Pose> ballList, ArrayList<Pose> buffer, ArrayList<ArrayList<Pose>> candidates, int index){

        if(buffer.size() == Math.min(4, ballList.size())){
            candidates.add(new ArrayList<>(buffer)); //add the buffer to canidates (as seperate instance) once we reach the end of the list
            return candidates; //return to the recursive call not main call
        }

        //cool thing is u can actually call a method INSIDE a method to do smth recursevly liek this
        //i lwk just learned this
        for(int i = index; i < ballList.size(); i++){
            buffer.add(ballList.get(i));

            permutationsGen(ballList, buffer, candidates, i + 1);

            buffer.remove(buffer.size() - 1); //after u get every combo from with this ball remove it from buffer
        }
        return candidates;
    }
    public ArrayList<Pose> costFunc(ArrayList<Pose> balls, Pose curpose){
        if (balls.isEmpty()) {
            return new ArrayList<>();
        }
        //generate every possible group of 4
        ArrayList<ArrayList<Pose>> canidateGroups = new ArrayList<>();// every group
        canidateGroups.addAll(permutationsGen(balls, new ArrayList<>(), new ArrayList<>(), 0));
        ArrayList<ArrayList<Pose>> candidateRoutes = new ArrayList<>();

        for(ArrayList<Pose> group : canidateGroups) {
            ArrayList<Pose> sorted = new ArrayList<>();
            group.sort(Comparator.comparingDouble(ball -> curpose.distance(ball))); //sort closest ball to bot
            sorted.add(group.remove(0));
            while (!group.isEmpty()) {
                //sort closest to last ball to get the best order of balls to get
                group.sort(Comparator.comparingDouble(ball -> sorted.get(sorted.size() - 1).distance(ball)));
                sorted.add(group.remove(0));
            }
            candidateRoutes.add(sorted);
        }
        //we end with all the sorted routes
        ArrayList<Routes> routes = new ArrayList<>();
        for (ArrayList<Pose> group : candidateRoutes) {
            ArrayList<Pose> route = new ArrayList<>(group.subList(0, Math.min(group.size(), 4)));
            double dist = curpose.distance(route.get(0));
            for (int i = 0; i < route.size() - 1; i++) {
                dist += route.get(i).distance(route.get(i + 1));//get the approx dist of each route
            }
            routes.add(new Routes(route, dist));
        }

        routes.sort(Comparator.comparingDouble(route -> route.score));//get route with best efficiency
        Collections.reverse(routes);

        return wallPadding(routes.get(0).balls, curpose); //get best route and add some wall padding


    }
    public ArrayList<Pose> wallPadding (ArrayList<Pose> balls, Pose curPose){
        //kinda scuffed but push the balls inward so the center of the bot goes to the fake pos but side of intake gets the actual ball
        ArrayList<Pose> bordered = new ArrayList<>();
        double padding = 7;//inches
        for(Pose ball : balls){
            if(ball.x() <= padding) {
                ball = ball.withX(padding);
            }else if(ball.x() >= 141.5-padding){
                ball = ball.withX(141.5-padding);
            }
            if(ball.y() <= 7){
                ball = ball.withY(padding);
            } else if(ball.y() >= 141.5-padding){
                ball = ball.withY(141.5-padding);
            }
            bordered.add(ball);
        }
        return bordered;
    }
    public ArrayList<Pose> combiner(ArrayList<Pose> route, Pose curPose){
        //combines clumps into 1 ball to make paths better
        if(route.size() <= 1) return route;
        //find slope of line from bot to first ball to find the heading the bot coming at
        for(int i = 0; i < route.size() - 1; i++){
            //just skip if balls not close
            if(route.get(i).distance(route.get(i+1)) > 16) continue;

            //get vector of ball to ball
            Vector2D ballToBall = route.get(i+1).toVector2D().minus(route.get(i).toVector2D());

            //get heading of bot when going to balls
            double theta;
            if(i == 0) {
                theta = Math.atan2(route.get(i).y() - curPose.y(), route.get(i).x() - curPose.x());
            }else{
                theta = Math.atan2(route.get(i).y() - route.get(i - 1).y(), route.get(i).x() - route.get(i - 1).x());
            }

            //get component of ball vector on bot heading for horiz and vert
            //unforunetly pedro removed projection func so gotta do manually

            Vector2D botVec = Vector2D.polar(1, theta + Math.PI/2.0);
            double compx = botVec.dot(ballToBall); //usually u divide by magnitude but our mag is 1 so no need

            Vector2D botVecPerp = Vector2D.polar(1, theta);
            double compy = botVecPerp.dot(ballToBall);
            if(Math.abs(compx) < 10 && Math.abs(compy) < 6){ //if not vertically far but moderate horiz far. then clump
                double x = (route.get(i).x() + route.get(i + 1).x()) / 2.0;
                double y = (route.get(i).y() + route.get(i + 1).y()) / 2.0;
                route.set(i,new Pose(x, y));
                route.remove(i+1);
            }
        }
        return route;
    }
    class Routes {// i wasnt tryna have a list inside a list inside a list
        ArrayList<Pose> balls;
        double distance;
        double score;
        Routes(ArrayList<Pose> balls, double distance) {
            this.balls = balls;
            this.distance = distance;
            this.score = balls.size() / distance; //get efficency of the route
        }
    }
}
