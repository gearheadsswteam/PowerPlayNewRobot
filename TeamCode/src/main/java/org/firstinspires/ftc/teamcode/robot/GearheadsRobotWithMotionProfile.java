package org.firstinspires.ftc.teamcode.robot;
//import com.outoftheboxrobotics.photoncore.PhotonCore;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

public class GearheadsRobotWithMotionProfile {

    public SampleMecanumDrive drive;
    public ElevatorWithMotionProfile elevator;
    public Arm arm;
    public Claw claw;

    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;


    public Servo retract;

    public RevColorSensorV3 holder;
    public IMU gyro;

    List<LynxModule> allHubs;


    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);

        retract = hwMap.get(Servo.class, "retract");

        holder = hwMap.get(RevColorSensorV3.class, "holder");

        gyro = hwMap.get(IMU.class, "gyro");

        allHubs = hwMap.getAll(LynxModule.class);



        for (LynxModule hub: allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //PhotonCore.enable();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);

        initializeElevator(hwMap);
        initializeArm(hwMap);
        initializeDriveTrain(hwMap);
        initializeClaw(hwMap);
    }
    public double getHeading() {
        return gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public void initializeElevator(HardwareMap hwMap){
        elevator = new ElevatorWithMotionProfile(hwMap);
        elevator.initialize();
    }

    public void initializeArm(HardwareMap hwMap){
        arm = new Arm(hwMap);
        arm.initialize();
    }

    public void initializeClaw(HardwareMap hwMap){
        claw = new Claw(hwMap);
        claw.initialize();
    }

    public void initializeDriveTrain(HardwareMap hwMap){
        fl = hwMap.get(DcMotorEx.class, "fl");
        fr = hwMap.get(DcMotorEx.class, "fr");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setDrivePowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void update(double curTime){
        //Move elevator
        boolean hasElevatorReached = this.elevator.hasElevatorReached();
        if(!hasElevatorReached){
            elevator.moveElevatorToHeight(curTime);
        }else{
            elevator.stopElevator();
            elevator.resetElevator();
        }
    }
}
