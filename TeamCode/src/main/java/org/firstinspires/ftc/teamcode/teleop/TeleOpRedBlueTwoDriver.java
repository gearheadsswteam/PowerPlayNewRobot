package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.holderDetectionThreshold;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftGroundClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftHighClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftLowClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftMedClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.odoUp;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.side;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.ValueStorage;
import org.firstinspires.ftc.teamcode.robot.GearheadsRobot;

@TeleOp(name = "TeleOpRedBlueTwoDriver")
public class TeleOpRedBlueTwoDriver extends LinearOpMode {
    GearheadsRobot robot = new GearheadsRobot();

    int holderDetectionCount = 0;
    double initialHeading = ValueStorage.lastPose.getHeading() - side * PI / 2;
    double robotHeading;
    double moveAngle;
    double moveMagnitude;
    double turn;
    double stateTime = 0;
    double time;
    boolean stateDir = true;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean aReleased = true;
    boolean bReleased = true;
    boolean xPressed = false;
    boolean xReleased = false;
    boolean yPressed = false;
    boolean yReleased = false;
    boolean lbPressed = false;
    boolean lbReleased = false;
    boolean rbPressed = false;
    boolean rbReleased = false;
    ElapsedTime clock = new ElapsedTime();


    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME = 500;

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }

    ;

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_START;

    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    public void initialize() {
        // hardware initilization code
        robot.init(hardwareMap);
        robot.retract.setPosition(odoUp);
        liftTimer.reset();
    }


    @Override
    public void runOpMode() {
        initialize();

        while (!isStarted() && !isStopRequested()) {
            robot.update(clock.seconds());
        }
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                aPressed = aReleased;
                aReleased = false;
            } else {
                aPressed = false;
                aReleased = true;
            }
            if (gamepad1.b) {
                bPressed = bReleased;
                bReleased = false;
            } else {
                bPressed = false;
                bReleased = true;
            }
            if (gamepad1.x) {
                xPressed = xReleased;
                xReleased = false;
            } else {
                xPressed = false;
                xReleased = true;
            }
            if (gamepad1.y) {
                yPressed = yReleased;
                yReleased = false;
            } else {
                yPressed = false;
                yReleased = true;
            }
            if (gamepad1.left_bumper) {
                lbPressed = lbReleased;
                lbReleased = false;
            } else {
                lbPressed = false;
                lbReleased = true;
            }
            if (gamepad1.right_bumper) {
                rbPressed = rbReleased;
                rbReleased = false;
            } else {
                rbPressed = false;
                rbReleased = true;
            }
            if (gamepad2.ps) {
                initialHeading -= robotHeading;
            }
            if (robot.holder.getDistance(DistanceUnit.INCH) < holderDetectionThreshold) {
                holderDetectionCount++;
            } else {
                holderDetectionCount = 0;
            }

            time = clock.seconds();

            switch (liftState) {
                case LIFT_START:
                    if (aPressed) {
                        robot.extendLiftProfile(time, liftLowClose[0], 0);
                        stateTime = robot.restTime();
                    } else if (bPressed) {
                        robot.extendLiftProfile(time, liftMedClose[0], 0);
                        stateTime = robot.restTime();
                    } else if (yPressed) {
                        robot.extendLiftProfile(time, liftHighClose[0], 0);
                        stateTime = robot.restTime();
                    } else if (xPressed) {
                        robot.extendLiftProfile(time, liftGroundClose[0], 0);
                        stateTime = robot.restTime();
                    }
                    liftState = LiftState.LIFT_EXTEND;
                    break;


                case LiftState.LIFT_EXTEND:
                    // check if the lift has finished extending,
                    // otherwise do nothing.
                    if (Math.abs(robot.liftL.getCurrentPosition() - robot.liftProfile.getFinalX()) < 10) {
                        // our threshold is within
                        // 10 encoder ticks of our target.
                        // this is pretty arbitrary, and would have to be
                        // tweaked for each robot.

                        // set the lift dump to dump
                        //Drop cargo

                        liftTimer.reset();
                        liftState = LiftState.LIFT_DUMP;
                    }
                    break;
                case LiftState.LIFT_DUMP:
                    if (liftTimer.seconds() >= DUMP_TIME) {
                        // The robot waited long enough, time to start
                        // retracting the lift
                        //@TODO
                        //Drop Cargo here
                        liftState = LiftState.LIFT_RETRACT;
                    }
                    break;
                case LiftState.LIFT_RETRACT:
                    if (Math.abs(robot.liftL.getCurrentPosition() - LIFT_LOW) < 10) {
                        liftState = LiftState.LIFT_START;
                    }
                    break;
                default:
                    // should never be reached, as liftState should never be null
                    liftState = LiftState.LIFT_START;
            }
        }


        robot.update(time);

        robotHeading = robot.getHeading() + initialHeading;
        moveAngle = atan2(-gamepad2.left_stick_x, -gamepad2.left_stick_y) - robotHeading;
        moveMagnitude = abs(pow(gamepad2.left_stick_x, 3)) + abs(pow(gamepad2.left_stick_y, 3));
        if (moveMagnitude < 0.01) {
            moveMagnitude = 0;
        }
        turn = pow(gamepad2.right_stick_x, 3);

        if (gamepad2.right_trigger < 0.1) {
            robot.setDrivePowers(moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn,
                    moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn,
                    moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn,
                    moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
        } else {
            robot.setDrivePowers((moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn) / 4,
                    (moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn) / 4,
                    (moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn) / 4,
                    (moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn) / 4);
        }
    }
}
}