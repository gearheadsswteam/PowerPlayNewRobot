package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.GearheadsRobot;
import org.firstinspires.ftc.teamcode.vision.SignalDetector;
public abstract class AbstractAutonomous extends LinearOpMode {
    public GearheadsRobot robot = new GearheadsRobot();
    SignalDetector detector;
    int runCase = 2;
    int caseDetected = 2;
    int caseDetectionLength = 0;
    ElapsedTime clock = new ElapsedTime();
    double time;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.retract.setPosition(odoDown);
        detector = new SignalDetector(hardwareMap);
        detector.init();
        robot.drive.setPoseEstimate(initPose());
        initialize();
//        robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (!isStarted() && !isStopRequested()) {
            if (detector.getCaseDetected() == caseDetected) {
                caseDetectionLength++;
            } else if (detector.getCaseDetected() > 0) {
                caseDetected = detector.getCaseDetected();
                caseDetectionLength = 1;
            }
            if (caseDetectionLength >= signalMinCount) {
                runCase = caseDetected;
            }
            //robot.update(clock.seconds());
            telemetry.addData("Case Detected", caseDetected);
            telemetry.addData("Case to Run", runCase);
            telemetry.update();
        }
        detector.end();
        side = side();
        run();
        lastPose = robot.drive.getPoseEstimate();
    }
    public abstract void initialize();
    public abstract void run();
    public abstract int side();
    public abstract Pose2d initPose();
}
