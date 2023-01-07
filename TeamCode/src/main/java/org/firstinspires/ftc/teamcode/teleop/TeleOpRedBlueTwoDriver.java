package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.elevatorGround;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.elevatorHigh;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.elevatorLow;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.elevatorMed;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.holderDetectionThreshold;
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
    double time;

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

    boolean coneAvailable = false;

    ElapsedTime clock = new ElapsedTime();


    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME = 500;

    public enum LiftState {
        LIFT_INIT,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }

    public enum ArmClawState {
        ARM_INIT_CLAW_OPEN,
        ARM_GRAB_CLAW_OPEN,
        ARM_GRAB_CLAW_CLOSED,
        ARM_INIT_CLAW_CLOSED
    }

    private int elevatorHeightNeeded = -1;
    private double armPositonNeeded = -1;
    private double clawPositonNeeded = -1;

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_INIT;

    //Arm State
    ArmClawState armState = ArmClawState.ARM_INIT_CLAW_OPEN;

    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime clawTimer = new ElapsedTime();

    public void initialize() {
        // hardware initilization code
        robot.init(hardwareMap);
        robot.retract.setPosition(odoUp);
        liftTimer.reset();

        //Move elevator to ground position
        while(robot.elevator.isElevatorAtInitPosition()) {
            robot.elevator.moveElevatorToHeight(elevatorGround);
        }

        //Move arm & claw to Init/open position
        robot.arm.moveArmToInitPosition();
        robot.claw.openClaw();
        armState = ArmClawState.ARM_INIT_CLAW_OPEN;
    }


    @Override
    public void runOpMode() {
        initialize();

        while (!isStarted() && !isStopRequested()) {
            sleep(200);
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

            //execute arm claw state machine
            executeArmStateMachine(armState);

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

    /**
     * By pressing Right Bumper we move the arm and claw from init position to grab to init
     * In init position with cone secured, it can then execute elevator state machine
     * @param armClawStateVal state of the arm & claw
     */
    private void executeArmStateMachine(ArmClawState armClawStateVal){
        switch (armClawStateVal) {
            case ARM_INIT_CLAW_OPEN:
                if (rbPressed) {
                    armPositonNeeded = ValueStorage.armGrabPositon;

                    robot.arm.moveArmToPosition(armPositonNeeded);
                    armState = ArmClawState.ARM_GRAB_CLAW_OPEN;
                }
                coneAvailable = false;
                break;

            case ARM_GRAB_CLAW_OPEN:
                // check if arm has moved to grab position
                if (Math.abs(robot.arm.getCurrentPosition() - armPositonNeeded) < 0.1) {
                    // our threshold is within
                    armPositonNeeded = -1;
                    if (rbPressed) {
                        robot.claw.closeClaw();
                        armState = ArmClawState.ARM_GRAB_CLAW_CLOSED;
                    }
                } else {//wait till arm has reached the position
                    //waiting
                }
                coneAvailable = false;
                break;


            case ARM_GRAB_CLAW_CLOSED:
                // check if the claw is closed
                if (robot.claw.isClawClosed()) {
                    // our threshold is within
                    coneAvailable = true;
                    clawPositonNeeded = -1;
                    if (rbPressed) {
                        armPositonNeeded = ValueStorage.armInitPosition;
                        robot.arm.moveArmToPosition(armPositonNeeded);
                        armState = ArmClawState.ARM_INIT_CLAW_CLOSED;
                    }

                } else {//wait till claw has closed
                    //waiting
                    coneAvailable = false;
                }
                break;


            case ARM_INIT_CLAW_CLOSED:
                // check if the claw has finished
                if (robot.claw.isClawClosed()) {
                    //Wait for Elevator command
                    executeElevatorStateMachine(liftState);
                } else {//wait till arm has reached the position
                    //waiting
                }
                coneAvailable = true;
                break;
        }

    }

    /**
     * By pressing a,b,x & y the elevator moves to different height states
     * By pressing left bumper the cone is dropped
     * @param liftStateVal
     */
    private void executeElevatorStateMachine(LiftState liftStateVal) {
        if(armState != ArmClawState.ARM_INIT_CLAW_CLOSED && !coneAvailable){
            //Error we are in wrong state
            return;
        }

        switch (liftStateVal) {
            case LIFT_INIT:
                if (aPressed) {
                    elevatorHeightNeeded = elevatorLow;
                    liftState = LiftState.LIFT_EXTEND;

                } else if (bPressed) {
                    elevatorHeightNeeded = elevatorMed;
                    liftState = LiftState.LIFT_EXTEND;

                } else if (yPressed) {
                    elevatorHeightNeeded = elevatorHigh;
                    liftState = LiftState.LIFT_EXTEND;

                } else if (xPressed) {
                    elevatorHeightNeeded = elevatorGround;
                    liftState = LiftState.LIFT_EXTEND;
                }

                //Need to do this for all heights
                if (liftState == LiftState.LIFT_EXTEND) {
                    robot.elevator.moveElevatorToHeight(elevatorHeightNeeded);
                }
                break;

            case LIFT_EXTEND:
                // check if the lift has finished extending,
                if (Math.abs(robot.elevator.getCurrentHeight() - elevatorHeightNeeded) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    robot.elevator.stopElevator();
                    robot.elevator.resetElevator(); //reset the PID controller
                    liftTimer.reset();
                    elevatorHeightNeeded = -1;


                    liftState = LiftState.LIFT_DUMP;
                } else {//Keep moving the elevator to desired height
                    robot.elevator.moveElevatorToHeight(elevatorHeightNeeded);
                }
                break;

            case LIFT_DUMP:
                if(lbPressed) {
                    robot.claw.openClaw();//Dump the cone
                    armState = ArmClawState.ARM_INIT_CLAW_OPEN;
                    coneAvailable = false;
                }

                if (liftTimer.seconds() >= DUMP_TIME && !coneAvailable) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    elevatorHeightNeeded = elevatorGround;
                    robot.elevator.moveElevatorToHeight(elevatorHeightNeeded);
                    liftTimer.reset();

                    armState = ArmClawState.ARM_INIT_CLAW_OPEN;
                    liftState = LiftState.LIFT_RETRACT;
                }
                break;

            case LIFT_RETRACT:
                if (Math.abs(robot.elevator.getCurrentHeightToAchieve() - elevatorHeightNeeded) < 10) {
                    robot.elevator.stopElevator();
                    robot.elevator.resetElevator(); //reset the PID controller
                    liftTimer.reset();
                    elevatorHeightNeeded = -1;

                    armState = ArmClawState.ARM_INIT_CLAW_OPEN;
                    liftState = LiftState.LIFT_INIT;
                } else {
                    robot.elevator.moveElevatorToHeight(elevatorHeightNeeded);
                }
                coneAvailable = false;
                break;
            default:
                // should never be reached, as liftState should never be null
                liftState = LiftState.LIFT_INIT;
        }
    }
}


