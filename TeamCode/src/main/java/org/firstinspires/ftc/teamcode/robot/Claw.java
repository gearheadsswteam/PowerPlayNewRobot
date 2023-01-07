package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.ValueStorage;

public class Claw {

    HardwareMap hwMap;
    private Servo leftServo;
    private Servo rightServo;

    private double currentPositionToSet;



    public Claw(HardwareMap hardwareMapMap) {
        hwMap = hardwareMapMap;
    }

    public void initialize() {
        leftServo = hwMap.get(Servo.class, "lClawServo");
        rightServo = hwMap.get(Servo.class, "rClawServo");
        rightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void closeClaw(){
        moveArmToPosition(ValueStorage.clawClosed);
    }

    public boolean isClawClosed(){
        return (Math.abs(getCurrentPosition() - ValueStorage.clawClosed) < 0.1);
    }
    public void openClaw(){
        moveArmToPosition(ValueStorage.clawOpen);
    }

    private void moveArmToPosition(double positionToMoveTo) {
        leftServo.setPosition(positionToMoveTo);
        rightServo.setPosition(1-positionToMoveTo);
        this.currentPositionToSet = positionToMoveTo;
    }

    public double getCurrentPosition() {
        return leftServo.getPosition();
    }

    public double getPositionToMoveTo() {
        return currentPositionToSet;
    }
}
