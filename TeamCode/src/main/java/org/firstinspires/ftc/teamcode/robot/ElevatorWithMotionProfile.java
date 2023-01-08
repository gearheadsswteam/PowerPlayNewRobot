package org.firstinspires.ftc.teamcode.robot;


import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.classes.ValueStorage;

public class ElevatorWithMotionProfile {
    private static final int ELEVATOR_HEIGHT_TOLERANCE = 5;
    private HardwareMap hwMap;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private PIDFController controller;
    private int elevatorHeightNeeded;
    private MotionProfile profile;
    private double profileStartTime;

    public static double MAX_VEL = 1400; // clicks/s
    public static double MAX_ACCEL = 3000; // clicks/s^2
    public static double MAX_JERK = 6000; // clicks/s^3


    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    private NanoClock clock = NanoClock.system();

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
    }

    public void setDesiredHeight(int desiredHeight) {
        //here
        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(elevatorHeightNeeded, 0, 0, 0);
        MotionState goal = new MotionState(desiredHeight, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );
        profileStartTime = clock.seconds();

        this.elevatorHeightNeeded = desiredHeight;
    }


    public void moveElevatorToHeight(double elapsedTime) {
        double power;
        double currentHeight = getCurrentHeight();
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            controller.setTargetVelocity(state.getV());
            controller.setTargetAcceleration(state.getA());
            power = controller.update(currentHeight);
        } else {
            // just hold the position
            controller.setTargetPosition(elevatorHeightNeeded);
            power = controller.update(currentHeight);
        }
        setPower(power);
    }

    private boolean isElevatorAtInitPosition() {
        return (Math.abs(getCurrentHeight() - ValueStorage.elevatorGround) < 10);
    }


    private int getCurrentHeight() {
        return leftMotor.getCurrentPosition();
    }

   //TODO don't we have to reset the controller after every move?
    public void resetElevator() {
        controller.reset();
        //No op profile
        profile = null;
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

}






