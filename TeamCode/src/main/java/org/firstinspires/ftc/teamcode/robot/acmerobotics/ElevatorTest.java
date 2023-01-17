package org.firstinspires.ftc.teamcode.robot.acmerobotics;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Simple test of motion-profiled elevator autonomous operation. The elevator should move *smoothly*
 * between random heights.
 */
@Autonomous(name = "ElevatorTest 2", group = "elevator")
public class ElevatorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevator elevator = new Elevator(hardwareMap);
        NanoClock clock = NanoClock.system();

        waitForStart();

        if (isStopRequested()) return;

        int count = 1;
        while (!isStopRequested()) {
            double targetHeight = Elevator.MAX_HEIGHT * Math.random();
            elevator.setHeight(targetHeight);
            telemetry.addData("Target Height " + targetHeight,"");

            double startTime = clock.seconds();
            while (!isStopRequested() && (clock.seconds() - startTime) < 5) {
                elevator.update();
            }
            int curPositon = elevator.getPosition();
            double curHeight = elevator.getCurrentHeight();
            double offSet = elevator.getOffset();
            telemetry.addData(count++ + " Position " + curPositon + " Height " + curHeight + " offset " + offSet, "");
            telemetry.update();
        }
    }
}