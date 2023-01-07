package org.firstinspires.ftc.teamcode.robot;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.classes.ValueStorage;

public class Elevator {
    HardwareMap hwMap;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    PIDFController pidf = new PIDFController(0.05, 0, 0, 0);
    private int currentHeightToSet;
    private static int INIT_POSITION = 0;


    public Elevator(HardwareMap hardwareMapMap) {
        hwMap = hardwareMapMap;
    }

    public void initialize() {
        leftMotor = hwMap.get(DcMotor.class, "lElMotor");
        rightMotor = hwMap.get(DcMotor.class, "rElMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveElevatorToHeight(int heightToSet) {
        currentHeightToSet = heightToSet;
        double output = pidf.calculate(
                leftMotor.getCurrentPosition(), heightToSet);
        leftMotor.setPower(output);
        rightMotor.setPower(output);
    }


    public boolean isElevatorAtInitPosition(){
        return (Math.abs(getCurrentHeight() - ValueStorage.elevatorGround) < 10);
    }


    public void stopElevator() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public int getCurrentHeight() {
        return leftMotor.getCurrentPosition();
    }

    public int getCurrentHeightToAchieve() {
        return currentHeightToSet;
    }

    public void resetElevator() {
        pidf.reset();
    }
}






