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
    HardwareMap hwMap;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    PIDFController controller;
    private int elevatorHeightNeeded;
    private MotionProfile profile;
    public static double liftMaxAccel = 3000;
    public static double liftMaxVel = 1400;


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
        controller = new PIDFController(coeffs);
    }

    public void moveElevatorToHeight(int heightToSet, double elapsedTime) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(heightToSet, 0, 0),
                liftMaxVel,
                liftMaxAccel,
                100
        );

        MotionState state = profile.get(elapsedTime);

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        double correction = controller.update(leftMotor.getCurrentPosition());
        elevatorHeightNeeded = heightToSet;
        leftMotor.setPower(correction);
        rightMotor.setPower(correction);
    }

    public void moveElevatorUntilHeightReached(double elapsedTime) {
        moveElevatorToHeight(elevatorHeightNeeded, elapsedTime);
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
    }

    public boolean hasElevatorReached() {
        return (Math.abs(getCurrentHeightNeeded() - getCurrentHeight()) < 10);
    }
}






