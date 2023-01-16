package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.INTAKE_POWER_AUTO;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armIn;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.autonomousArmProfile;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.autonomousWristProfile;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardArmProfile1;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardArmProfile2;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardWristProfile1;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardWristProfile2;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.gripperHold;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.gripperRelease;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.holderDetectionThreshold;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftMaxAccel;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftMaxVel;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftMedClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.rollerDown;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.rollerRetract;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.rollerUp;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.sides;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.wristIn;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.TrapezoidalProfile;
import org.firstinspires.ftc.teamcode.classes.ValueStorage;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Right3Stack", group = "Right")
public class AutonomousRight3Stack extends AbstractAutonomous {
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
    boolean traj2Started = false;
    boolean traj4Started = false;
    boolean traj5Done = false;
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
                }).build();

        //Drop point to stack
        traj2 = robot.drive.trajectorySequenceBuilder(dropPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                .splineTo(stackPose.vec(), stackPose.getHeading() + PI)
                .lineTo(knockPose.vec())
                .addDisplacementMarker(() -> {
                    //Intake cone
                    usingSensor = true;
                    robot.elevator.setDesiredHeight(ValueStorage.elevatorStackPickup);
                    traj2Started = true;
                    //robot.roller.setPosition(rollerDown);
                    //robot.setIntakePowers(INTAKE_POWER_AUTO, INTAKE_POWER_AUTO);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(intakePose.vec())
                .addTemporalMarker(1, 0, () -> {
                    intakeTrajDone = true;
                })
                .build();

        //Stack to the drop point
        traj3 = robot.drive.trajectorySequenceBuilder(backPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, 0, () -> {
                    endDropTraj = true;
                    dropTrajDone = true;
                    dropTrajTime = time;
                    cycles++;
                })
                .build();

        //Drop point to stack for the 2nd cone
        traj4 = robot.drive.trajectorySequenceBuilder(dropPose)
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
                })
                .build();

        //Drop point to park
        traj5 = new TrajectorySequence[] {
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
        //robot.armProfile = autonomousArmProfile(0);
        robot.arm.moveArmToDropPosition();
        //.addExtendTrapezoidal(armMaxVel, armMaxAccel, liftHighClose[1], 0);
        //robot.wristProfile = autonomousWristProfile(0);
        //.addExtendTrapezoidal(wristMaxVel, wristMaxAccel, liftHighClose[2],0);

        robot.drive.followTrajectorySequenceAsync(traj1);

        while (opModeIsActive() && !isStopRequested() && (!traj5Done || time > retractTime)) {
            time = clock.seconds();


            //Starting the lift in Traj 1 & Traj 3...as you are going to the drop point

            //You are at drop position
            if (endDropTraj) {
                robot.claw.openClaw();
                //robot.gripper.setPosition(gripperRelease);
                endDropTraj = false;
            }

            if(traj2Started){
                if(!robot.elevator.isBusy()){
                    traj2Started = false;
                    robot.claw.closeClaw();
                }
            }

            if(traj4Started){
                if(!robot.elevator.isBusy()){
                    traj4Started = false;
                    robot.claw.closeClaw();

            }


            //Execute Traj 2 from Drop position to stack
            //Drop the stack
            //GO back until cone is intaken
            //Give 1 second for cone to drop
            if (dropTrajDone && time - dropTrajTime > 0.5) {
                if (cycles == 0) {
                    robot.drive.followTrajectorySequenceAsync(traj2);
                    //Retract the lift
                    robot.elevator.setDesiredHeight(ValueStorage.elevatorLow);
//                    robot.extendLiftProfile(time, 0, 0);
//                    robot.extendArmProfile(time, armIn, 0);
//                    robot.extendWristProfile(time, wristIn, 0);
//                    robot.roller.setPosition(rollerUp);
//                    robot.setIntakePowers(0, 0.5);
//                    retractTime = robot.restTime();
                    dropTrajDone = false;
                } else if (cycles < 3) {
                    robot.drive.followTrajectorySequenceAsync(traj4);
                    robot.elevator.setDesiredHeight(ValueStorage.elevatorLow);

//                    robot.extendLiftProfile(time, 0, 0);
//                    robot.extendArmProfile(time, armIn, 0);
//                    robot.extendWristProfile(time, wristIn, 0);
//                    retractTime = robot.restTime();
                    retractDone = true;
                    dropTrajDone = false;
                } else {//Two cycles for three cones done...go to park
                    robot.drive.followTrajectorySequenceAsync(traj5[runCase - 1]);
//                    robot.setIntakePowers(0, 0);
//                    robot.extendLiftProfile(time, 0, 0);
//                    robot.extendArmProfile(time, armIn, 0);
//                    robot.extendWristProfile(time, wristIn, 0);
//                    retractTime = robot.restTime();
                    retractDone = true;
                    dropTrajDone = false;
                }
            }

            //**
            // Cone is inside
            //Go to drop position
            if (intakeTrajDone || (usingSensor && robot.holder.getDistance(DistanceUnit.INCH) < holderDetectionThreshold)) {//cone is inside
                TrajectorySequence tempTraj = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .lineTo(backPose.vec())
                        .addTemporalMarker(0, 0.5, () -> {
                            //robot.setIntakePowers(-0.5, -0.5);
                        })
                        .addTemporalMarker(1, 0, () -> {
                            robot.drive.followTrajectorySequenceAsync(traj3);
                        })
                        .build();
                robot.drive.followTrajectorySequenceAsync(tempTraj);
                robot.armProfile = forwardArmProfile1(time);
                robot.wristProfile = forwardWristProfile1(time);
                robot.liftProfile = new TrapezoidalProfile(liftMaxVel, liftMaxAccel, time + 1, 0, 0, liftMedClose[0], 0);
                robot.setIntakePowers(0, 0);
                robot.roller.setPosition(rollerRetract);
                robot.gripper.setPosition(gripperHold);
                usingSensor = false;
                intakeTrajDone = false;
                //Cone is inside
            }


            //You are at drop position

            /**
             if (traj3Done && time - traj3Time > 1) {
             //three cones
             if (cycles < 3) {
             robot.drive.followTrajectorySequenceAsync(traj4);
             } else {//Three cycles for three cones done...go to park
             robot.drive.followTrajectorySequenceAsync(traj5[runCase - 1]);
             robot.extendLiftProfile(time, 0, 0);
             robot.extendArmProfile(time, armIn, 0);
             robot.extendWristProfile(time, wristIn, 0);
             retractTime = robot.restTime();
             }
             traj3Done = false;
             }
             **/
            if (retractDone && time > retractTime) {
                robot.armProfile = forwardArmProfile2(time);
                robot.wristProfile = forwardWristProfile2(time);
                doneTime = robot.armTime();
                retractDone = false;
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
