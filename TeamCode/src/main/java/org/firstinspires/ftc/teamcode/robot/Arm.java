package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.ValueStorage;

public class Arm {

    HardwareMap hwMap;
    private Servo leftServo;
    private Servo rightServo;

    private double armPositonNeeded = - 1;


    public Arm(HardwareMap hardwareMapMap) {
        hwMap = hardwareMapMap;
    }

    public void initialize() {
        leftServo = hwMap.get(Servo.class, "lArmServo");
        rightServo = hwMap.get(Servo.class, "rArmServo");
        rightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void moveArmToPosition(double positionToMoveTo) {
        leftServo.setPosition(positionToMoveTo);
        rightServo.setPosition(1-positionToMoveTo);
        this.armPositonNeeded = positionToMoveTo;
    }

    public void moveArmToGrabPosition(){
        moveArmToPosition(ValueStorage.armGrabPositon);
    }

    public void moveArmToDropPosition(){
        moveArmToPosition(ValueStorage.armDropPosition);
    }

    //public void moveArmToInitPosition(){
    //    moveArmToPosition(ValueStorage.armInitPosition);
    //}

//    public void moveArmToGroundPosition(){
//        moveArmToPosition(ValueStorage.armGroundPosition);
//    }

    public double getCurrentPosition() {
        return leftServo.getPosition();
    }

    public double getPositionToMoveTo() {
        return armPositonNeeded;
    }

    public boolean hasArmReachedNeededPosition(){
        return (Math.abs(armPositonNeeded - getCurrentPosition()) < 10);
    }
}
