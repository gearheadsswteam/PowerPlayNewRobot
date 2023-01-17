package org.firstinspires.ftc.teamcode.robot.acmerobotics;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Simple test of motion-profiled elevator autonomous operation. The elevator should move *smoothly*
 * between random heights.
 */
@Autonomous(group = "elevator")
public class ElevatorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevator elevator = new Elevator(hardwareMap);
        NanoClock clock = NanoClock.system();

        waitForStart();

        if (isStopRequested()) return;

        double currentHeight = elevator.getCurrentHeight();

        double curHeightToDisplay = elevator.getCurrentHeight();
        telemetry.addData("Current  Height " + curHeightToDisplay,"");
        telemetry.update();

        //sleep(3000);

        elevator.setHeight(Elevator.MAX_HEIGHT);
        while (elevator.isBusy()) {
            elevator.update();
        }

        curHeightToDisplay = elevator.getCurrentHeight();
        telemetry.addData("Current  Height " + curHeightToDisplay,"");
        telemetry.update();

        //sleep(3000);

        elevator.setHeight(currentHeight);
        while (elevator.isBusy()) {
            elevator.update();
        }

        curHeightToDisplay = elevator.getCurrentHeight();
        telemetry.addData("Current  Height " + curHeightToDisplay,"");
        telemetry.update();
    }
}
