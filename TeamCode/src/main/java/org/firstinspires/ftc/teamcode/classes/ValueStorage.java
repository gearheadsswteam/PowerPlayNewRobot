package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import static com.qualcomm.robotcore.util.Range.*;

public class ValueStorage {
    public static int holderMinCount = 20;
    public static int signalMinCount = 10;
    public static double holderDetectionThreshold = 0.75;
    public static double odoUp = 0.22;
    public static double odoDown = 0.47;


    public static int elevatorGround = 0;
    public static int elevatorStackPickupInit = 175;
    public static int elevatorStackPickup = 175;
    public static int elevatorLow = 175;
    public static int elevatorMed = 600;
    public static int elevatorHigh = 1025;

    public static double clawOpen = 0;
    public static double clawClosed = 0.5;

    public static double armDropPosition = 0.5;
    public static double armGrabPositon = 0.5;


    public static double clawOpenTime = 500;
    public static double clawClosedTime = 500;

    public static int side = sides.RED;

    public static class sides {
        public static final int RED = 1;
        public static final int BLUE = -1;
    }

    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}