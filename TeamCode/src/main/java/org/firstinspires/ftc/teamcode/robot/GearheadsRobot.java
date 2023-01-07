package org.firstinspires.ftc.teamcode.robot;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
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
import org.firstinspires.ftc.teamcode.classes.PidfController;
import org.firstinspires.ftc.teamcode.classes.TrapezoidalProfile;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.List;
public class GearheadsRobot {

    public SampleMecanumDrive drive;

    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;

    public DcMotorEx liftL;
    public DcMotorEx liftR;

    public Servo retract;

    public RevColorSensorV3 holder;
    public IMU gyro;

    List<LynxModule> allHubs;

    PidfController liftPidf = new PidfController(liftKp, liftKi, liftKd) {
        @Override
        public double kf(double input) {
            return liftKf (input);
        }
    };
    public TrapezoidalProfile liftProfile = new TrapezoidalProfile(liftMaxVel, liftMaxAccel, 0, 0, 0, 0, 0);

    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        fl = hwMap.get(DcMotorEx.class, "fl");
        fr = hwMap.get(DcMotorEx.class, "fr");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");

        liftL = hwMap.get(DcMotorEx.class, "liftL");
        liftR = hwMap.get(DcMotorEx.class, "liftR");

        retract = hwMap.get(Servo.class, "retract");

        holder = hwMap.get(RevColorSensorV3.class, "holder");

        gyro = hwMap.get(IMU.class, "gyro");

        allHubs = hwMap.getAll(LynxModule.class);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (LynxModule hub: allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //PhotonCore.enable();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);
    }
    public double getHeading() {
        return gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public void update(double time) {
        liftPidf.set(liftProfile.getX(time));
        liftPidf.update(liftL.getCurrentPosition());
        liftL.setPower(liftPidf.get());
        liftR.setPower(liftPidf.get());
    }
    public void setDrivePowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void extendLiftProfile(double t, double xf, double vf) {
        liftProfile = liftProfile.extendTrapezoidal(t, xf, vf);
    }

    //TODO This needs to be fixed
    public double restTime() {
        return max(max(liftProfile.getTf(),liftProfile.getTf()),liftProfile.getTf());
    }
}
