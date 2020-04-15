package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This awesome program for Singularity Technology
 * was created by Albert on 2/6/2017.
 */

@Autonomous (name = "Auto")
public class Auto extends Methods {

    Hardware myrobot = new Hardware(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Hardware
        myrobot.init(hardwareMap);


        matchConfigure(myrobot);

        /* -----------Driver press play----------- */
        waitForStart();


        driveToSkyStone(myrobot);
        Hardware.skyStoneLocation = findSkyStone();
        skyStonePath(myrobot);
        moveFoundation(myrobot);


        // Automatically transition to TeleOp program
        AutoTransitioner.transitionOnStop(this, "teleOp");
    }
}