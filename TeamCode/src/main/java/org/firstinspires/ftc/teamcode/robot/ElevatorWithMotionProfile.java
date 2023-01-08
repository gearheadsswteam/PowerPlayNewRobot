package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.classes.ValueStorage;

public class ElevatorWithMotionProfile {
    public static final int ELEVATOR_HEIGHT_TOLERANCE = 5;
    HardwareMap hwMap;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    PIDFController controller;
    private int elevatorHeightNeeded;
    private MotionProfile profile;
    private double profileStartTime;
    public static double liftMaxAccel = 3000;
    public static double liftMaxVel = 1400;

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    private MotionProfile noopProfile;

    public ElevatorWithMotionProfile(HardwareMap hardwareMapMap) {
        hwMap = hardwareMapMap;
    }


    public void initialize() {
        leftMotor = hwMap.get(DcMotor.class, "lElMotor");
        rightMotor = hwMap.get(DcMotor.class, "rElMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDCoefficients coeffs = new PIDCoefficients(0.05, 0, 0);
        // create the controller
        controller = new PIDFController(coeffs, kV, kA, kStatic);
        controller.setOutputBounds(-1, 1);

        noopProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(leftMotor.getCurrentPosition(), 0, 0),
                new MotionState(leftMotor.getCurrentPosition(), 0, 0),
                liftMaxVel,
                liftMaxAccel,
                100);
    }

    public void setProfile(int heightToSet, double startTime) {
        //TODO: Add check to ensure the elevator is in the right init height

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(leftMotor.getCurrentPosition(), 0, 0),
                new MotionState(heightToSet, 0, 0),
                liftMaxVel,
                liftMaxAccel,
                100
        );

        profileStartTime = startTime;
        elevatorHeightNeeded = heightToSet;
    }


    public void moveElevatorToHeight(double elapsedTime) {
        MotionState state = profile.get(elapsedTime - profileStartTime);

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        double correction = controller.update(leftMotor.getCurrentPosition());

        leftMotor.setPower(correction);
        rightMotor.setPower(correction);
    }

    public double getProfileCompletionTime() {
        return profileStartTime + profile.duration();
    }


    public boolean isElevatorAtInitPosition() {
        return (Math.abs(getCurrentHeight() - ValueStorage.elevatorGround) < 10);
    }


    public void stopElevator() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public int getCurrentHeight() {
        return leftMotor.getCurrentPosition();
    }

    public int getCurrentHeightNeeded() {
        return elevatorHeightNeeded;
    }

    public void resetElevator() {
        controller.reset();
        //No op profile
        profile = noopProfile;
    }


    public boolean hasElevatorReached(double curtime) {
        //TODO is is better to use profile time or use the height tolerance also
        return ((Math.abs(getCurrentHeightNeeded() - getCurrentHeight()) < ELEVATOR_HEIGHT_TOLERANCE) && curtime > getProfileCompletionTime());
    }

}






