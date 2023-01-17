package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.sides;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.ValueStorage;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Right3StackOpti", group = "Right")
public class AutonomousRight3StackOpti extends AbstractAutonomous {
    Pose2d dropPose = new Pose2d(-40, 15, 0.45);
    Pose2d[] parkPose = {new Pose2d(-11, 11, 0), new Pose2d(-35, 11, 0), new Pose2d(-59, 11, 0)};
    Pose2d stackPose = new Pose2d(-60, 11, 0);
    Pose2d knockPose = new Pose2d(-50, 11, 0);
    Pose2d backPose = new Pose2d(-54, 11, 0);
    Pose2d intakePose = new Pose2d(-62, 11, 0);
    ElapsedTime clock = new ElapsedTime();
    double time = 0;
    double dropTrajTime = 1000;
    double retractTime = 1000;
    double doneTime = 1000;

    boolean endDropTraj = false;
    boolean dropTrajDone = false;

    boolean traj1Started = false;
    boolean traj1Ended = false;
    boolean traj1Completed = false;
    double traj1CompleteTime;
    double clawOpenTime;
    boolean clawOpenInitiated_1 = false;

    boolean traj2Started = false;
    boolean traj2Ended = false;
    boolean traj2Completed = false;
    double traj2CompleteTime;
    double elevatorStackPickupInitTime;

    boolean traj3Started = false;
    boolean traj3Ended = false;
    boolean traj3Completed = false;
    double traj3CompleteTime;
    boolean clawOpenInitiated_3 = false;

    boolean traj4Started = false;
    boolean traj4Ended = false;
    boolean traj4Completed = false;
    double traj4CompleteTime;

    boolean traj5Done = false;
    boolean traj5Started = false;
    boolean traj5Ended = false;
    boolean traj5Completed = false;
    double traj5CompleteTime;

    boolean intakeTrajDone = false;

    boolean retractDone = true;
    boolean usingSensor = false;

    int cycles = 0;

    TrajectorySequence traj1; //From start to drop point
    TrajectorySequence traj2; //From drop to stack
    TrajectorySequence traj3; //From stack to drop
    TrajectorySequence traj4; //From drop to stack
    TrajectorySequence[] traj5; //From drop to park


    @Override
    public void initialize() {
        //Start to drop point
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .addDisplacementMarker(0, () -> {
                    traj1Started = true;
                })
                .splineTo(new Vector2d(-35, 40), -PI / 2)
                .lineTo(new Vector2d(-35, 25))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToSplineHeading(dropPose, 2)
                .waitSeconds(0.3)
                .resetConstraints()
                .addTemporalMarker(1, -0.7, () -> {
                    // start lift
                    robot.elevator.setDesiredHeight(ValueStorage.elevatorMed);
                    //robot.extendLiftProfile(time, liftMedClose[0], 0);
                })
                .addTemporalMarker(1, 0, () -> {
                    endDropTraj = true;
                    dropTrajDone = true;
                    dropTrajTime = time;

                    traj1Started = false;
                    traj1Ended = true;
                    traj1Completed = true;
                }).build();

        //Drop point to stack
        traj2 = robot.drive.trajectorySequenceBuilder(dropPose)
                .addDisplacementMarker(0, () -> {
                    traj2Started = true;
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                .splineTo(stackPose.vec(), stackPose.getHeading() + PI)
                .lineTo(knockPose.vec())
                .addDisplacementMarker(() -> {
                    //Intake cone
                    usingSensor = true;
                    robot.elevator.setDesiredHeight(ValueStorage.elevatorStackPickupInit);
                    traj2Started = true;
                    //robot.roller.setPosition(rollerDown);
                    //robot.setIntakePowers(INTAKE_POWER_AUTO, INTAKE_POWER_AUTO);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(intakePose.vec())
                .addTemporalMarker(1, 0, () -> {
                    intakeTrajDone = true;
                    traj2Started = false;
                    traj2Ended = true;
                    traj2Completed = true;
                })
                .build();

        //Stack to the drop point
        traj3 = robot.drive.trajectorySequenceBuilder(backPose)
                .addDisplacementMarker(0, () -> {
                    traj3Started = true;
                    robot.elevator.setDesiredHeight(ValueStorage.elevatorHigh);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, 0, () -> {
                    endDropTraj = true;
                    dropTrajDone = true;
                    dropTrajTime = time;
                    cycles++;

                    traj3Started = false;
                    traj3Ended = true;
                    traj3Completed = true;
                })
                .build();

        //Drop point to stack for the 2nd cone
        traj4 = robot.drive.trajectorySequenceBuilder(dropPose)
                .addDisplacementMarker(0, () -> {
                    traj4Started = true;
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                .splineTo(backPose.vec(), backPose.getHeading() + PI)
                .addDisplacementMarker(() -> {
                    //Intake cone
                    usingSensor = true;
                    robot.elevator.setDesiredHeight(ValueStorage.elevatorStackPickup);
                    traj4Started = true;
                    //robot.roller.setPosition(rollerDown);
                    //robot.setIntakePowers(INTAKE_POWER_AUTO, INTAKE_POWER_AUTO);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(intakePose.vec())
                .addTemporalMarker(1, 0, () -> {
                    intakeTrajDone = true;

                    traj4Started = false;
                    traj4Ended = true;
                    traj4Completed = true;

                })
                .build();

        //Drop point to park
        traj5 = new TrajectorySequence[]{
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineToSplineHeading(parkPose[0], 0)
                        .addTemporalMarker(1, 0, () -> {
                            traj5Done = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .lineToLinearHeading(parkPose[1])
                        .addTemporalMarker(1, 0, () -> {
                            traj5Done = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[2].vec(), PI)
                        .addTemporalMarker(1, 0, () -> {
                            traj5Done = true;
                        })
                        .build()

        };
    }


    @Override
    public void run() {
        clock.reset();

        //Set up the arm init position
        robot.arm.moveArmToDropPosition();

        //Start Traj 1 and move elevator
        robot.drive.followTrajectorySequenceAsync(traj1);

        while (opModeIsActive() && !isStopRequested() && (!traj5Done || time > retractTime)) {
            time = clock.seconds();

            //If traj 1 is ended and elevator at right height
            if (traj1Ended) {//Driving complete
                if (!robot.elevator.isBusy() && !traj1Completed) {//Elevator has reached
                    if (!clawOpenInitiated_1) {//Claw open not started
                        robot.claw.openClaw();
                        clawOpenTime = clock.seconds();
                        clawOpenInitiated_1 = true;
                    } else {//Claw open was started
                        time = clock.seconds();
                        if (time - clawOpenTime > 1) {//Claw has had time to open & drop cone
                            traj1Completed = true;
                            traj1CompleteTime = time;
                        } else {
                            //wait for cone to drop
                        }
                    }
                } else {
                    //wait for elevator to complete
                }
            }

            if(traj1Completed & !traj2Started){
                //Start Traj 2 and move elevator to stack
                robot.drive.followTrajectorySequenceAsync(traj2);
            }

            if(traj2Ended){//if drivign complete
                if(!robot.elevator.isBusy()){//if elevator hovering over stack
                    //Move elevator to take top of the stack
                    robot.elevator.setDesiredHeight(ValueStorage.elevatorStackPickup);
                    elevatorStackPickupInitTime = clock.seconds();
                    //Give time to elevator to move to the top of stack
                    if(clock.seconds() - elevatorStackPickupInitTime> 1){
                        //Close the claw
                        if(robot.claw.isClawOpen()) {
                            robot.claw.closeClaw();
                        }else{
                            //If closed it means cone is grabbed, traj 2 is complete
                            traj2Completed = true;
                        }
                    }
                }
            }

            if(traj2Completed && !traj3Started){
                //Start driving to drop point and start elevator
                robot.drive.followTrajectorySequenceAsync(traj3);
            }

            if(traj3Started & !traj3Completed){
                if (traj3Ended) {//Driving complete
                    if (!robot.elevator.isBusy() && !traj3Completed) {//Elevator has reached
                        if (!clawOpenInitiated_3) {//Claw open not started
                            robot.claw.openClaw();
                            clawOpenTime = clock.seconds();
                            clawOpenInitiated_3 = true;
                        } else {//Claw open was started
                            time = clock.seconds();
                            if (time - clawOpenTime > 1) {//Claw has had time to open & drop cone
                                traj3Completed = true;
                                traj3CompleteTime = time;
                            } else {
                                //wait for cone to drop
                            }
                        }
                    } else {
                        //wait for elevator to complete
                    }
                }
            }

            robot.drive.update();
            robot.update(time);
        }

    }

    @Override
    public int side() {
        return sides.BLUE;
    }

    @Override
    public Pose2d initPose() {
        return new Pose2d(-32, 60, -PI / 2);
    }

}
