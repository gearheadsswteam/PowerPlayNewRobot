package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RightPark-Safe", group = "Right")
public class AutonomousRightParkSafe extends AbstractAutonomous {
    Pose2d dropPose = new Pose2d(-36, 36, -0.75);
    Pose2d[] parkPose = new Pose2d[]{new Pose2d(-11, 34, -PI / 2), new Pose2d(-35, 34, -PI / 2), new Pose2d(-59, 34, -PI / 2)};
    TrajectorySequence traj1;
    TrajectorySequence[] traj2;
    ElapsedTime clock = new ElapsedTime();
    double time = 0;
    double traj1Time = 1000;
    double retractTime = 1000;
    double doneTime = 1000;
    boolean startLift = false;
    boolean endTraj1 = false;
    boolean traj1Done = false;
    boolean traj2Done = false;
    boolean retractDone = true;

    @Override
    public void initialize() {
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(-35, 50), -PI / 2)
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .resetConstraints()
                .addTemporalMarker(1, -1.2, () -> {
                    startLift = true;
                })
                .addTemporalMarker(1, 0, () -> {
                    endTraj1 = true;
                    traj1Done = true;
                    traj1Time = clock.seconds();
                }).build();
        traj2 = new TrajectorySequence[]{
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .lineToLinearHeading(parkPose[1])
                        .lineTo(parkPose[0].vec())
                        .addTemporalMarker(1, 0, () -> {
                            traj2Done = true;
                        }).build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .lineToLinearHeading(parkPose[1])
                        .addTemporalMarker(1, 0, () -> {
                            traj2Done = true;
                        }).build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .lineToLinearHeading(parkPose[1])
                        .lineTo(parkPose[2].vec())
                        .addTemporalMarker(1, 0, () -> {
                            traj2Done = true;
                        }).build(),
        };
    }

    @Override
    public void run() {
        clock.reset();

        while (opModeIsActive() && !isStopRequested() && (!traj2Done || time < doneTime)) {
            time = clock.seconds();
            robot.drive.update();
            //robot.update(time);
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
