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

        while (!isStopRequested()) {
            elevator.setHeight(Elevator.MAX_HEIGHT * Math.random());

            double startTime = clock.seconds();
            while (!isStopRequested() && (clock.seconds() - startTime) < 5) {
                elevator.update();
            }
        }
    }
}