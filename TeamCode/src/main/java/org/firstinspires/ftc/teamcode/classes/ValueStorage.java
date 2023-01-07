package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import static com.qualcomm.robotcore.util.Range.*;

public class ValueStorage {
    public static int holderMinCount = 20;
    public static int signalMinCount = 10;
    public static double holderDetectionThreshold = 0.75;
    public static double odoUp = 0.22;
    public static double odoDown = 0.47;


    public static double liftMaxAccel = 3000;
    public static double liftMaxVel = 1400;
    public static double liftKp = 0.05;
    public static double liftKi = 0;
    public static double liftKd = 0;

    public static double[] liftLowClose = {175};
    public static double[] liftMedClose = {600};
    public static double[] liftHighClose = {1025};
    public static double[] liftHighFar = {1100};
    public static double[] liftGroundClose = {0, 0.90, 0.20};
    public static double[] liftPickup = {0, 0.96, 0.20};

    public static double[] adjust(double liftPos, double increment) {
        double weight;
        double groundInterval = 0.2;
        double closeInterval = 0.9;
        if (liftPos < liftLowClose[0]) {
            weight = scale(liftPos, 0, liftLowClose[0], 0, groundInterval);
        } else if (liftPos < liftHighClose[0]) {
            weight = scale(liftPos, liftLowClose[0], liftHighClose[0], groundInterval, closeInterval);
        } else {
            weight = scale(liftPos, liftHighClose[0], liftHighFar[0], closeInterval, 1);
        }
        weight = clip(weight + increment, 0, 1);
        if (weight < groundInterval) {
            return new double[]{scale(weight, 0, groundInterval, 0, liftLowClose[0]),
                    scale(weight, 0, groundInterval, liftGroundClose[1], liftLowClose[1]),
                    scale(weight, 0, groundInterval, liftGroundClose[2], liftLowClose[2])};
        } else if (weight < closeInterval) {
            return new double[]{scale(weight, groundInterval, closeInterval, liftLowClose[0], liftHighClose[0]),
                    liftLowClose[1], liftLowClose[2]};
        } else {
            return new double[]{scale(weight, closeInterval, 1, liftHighClose[0], liftHighFar[0]),
                    scale(weight, closeInterval, 1, liftHighClose[1], liftHighFar[1]), liftHighClose[2]};
        }
    }

    public static double liftKf(double input) {
        return 0.0003 * input;
    }

    public static int side = sides.RED;

    public static class sides {
        public static final int RED = 1;
        public static final int BLUE = -1;
    }

    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}