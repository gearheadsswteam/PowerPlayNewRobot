package org.firstinspires.ftc.teamcode.robot.acmerobotics;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Simple test of motion-profiled elevator autonomous operation. The elevator should move *smoothly*
 * between random heights.
 */
@Autonomous(group = "elevator")
public class ElevatorInchesToTicks extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevator elevator = new Elevator(hardwareMap);
        NanoClock clock = NanoClock.system();

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            int curPositon = elevator.getPosition();
            double curHeight = elevator.getCurrentHeight();
            double offSet = elevator.getOffset();
            telemetry.addData("Position " + curPositon + " Height " + curHeight + " offset " + offSet, "");
            telemetry.update();

            //elevator.setPosition(curPositon + 1000);
            sleep(1000);
        }
    }
}
