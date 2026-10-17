package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Matrix;

public class CVConstants {
    public static final double CAMERA_HEIGHT = 9.1; // in from center of ball not ground to cam
    public static final double CAMERA_PITCH = Math.toRadians(0.0); //1.3 deg down cuz the mount was very sus
    public static final double FORWARD_OFFSET = 7.5; // in, acc offset is more like 5 but i want to get ball closer to bot

    public static final Matrix xyzConverter = new Matrix(
            //xyz of cam is diff from xyz of bot
            // y axis of bot is acc z axis of cam
            new double[][] {
               {1, 0, 0}, //keep x to x
                {0, 0, 1}, // y to Z
                {0, 1, 0}, // z to y
        });

    public static Matrix createRotationX(double angle) {
        //for adding pitch we rotate around the x axis of camera
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return new Matrix(
                new double[][] {
                        {1, 0,  0},
                        {0, c, -s},
                        {0, s,  c},
                }
        );
    }

}
