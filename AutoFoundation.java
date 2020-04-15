package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * This awesome program for Singularity Technology
 * was created by Albert on 2/6/2017.
 */

@Autonomous (name = "Auto Foundation")
public class AutoFoundation extends Methods {

    Hardware myrobot = new Hardware(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Hardware
        myrobot.init(hardwareMap);

        matchConfigure(myrobot);

        Hardware.driveDirection driveDirectionOne = Hardware.driveDirection.LEFT;
        Hardware.driveDirection driveDirectionTwo = Hardware.driveDirection.RIGHT;
        if (Hardware.side == Hardware.sideOptions.BLUE) {
            driveDirectionOne = Hardware.driveDirection.LEFT;
            driveDirectionTwo = Hardware.driveDirection.RIGHT;
        }
        if (Hardware.side == Hardware.sideOptions.RED) {
            driveDirectionOne = Hardware.driveDirection.RIGHT;
            driveDirectionTwo = Hardware.driveDirection.LEFT;
        }

        /* -----------Driver press play----------- */
        waitForStart();

        if (!(Hardware.side == Hardware.sideOptions.NULL)) {
            drive(myrobot, 25, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
            drive(myrobot, 5.5, Hardware.driveSpeed, driveDirectionOne);
            drive(myrobot, 5, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
            drive(myrobot, 3, 0.3, Hardware.driveDirection.FORWARD);
            //dropFoundation(myrobot);
            drive(myrobot, 32, Hardware.driveSpeed, Hardware.driveDirection.BACKWARD);
            raiseFoundation(myrobot);
            drive(myrobot, 34, Hardware.driveSpeed, driveDirectionTwo);
            drive(myrobot, 20, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
            encoderTurn(myrobot, 0, Hardware.turnSpeed);
            drive(myrobot, 25, Hardware.driveSpeed, driveDirectionOne);
            encoderTurn(myrobot, 0, Hardware.turnSpeed);
            drive(myrobot, 28, Hardware.driveSpeed, driveDirectionTwo);
        }
        else {
            drive(myrobot, 5, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
        }

        // Automatically transition to TeleOp program
        AutoTransitioner.transitionOnStop(this, "teleOp");
    }
}