package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This awesome program for Singularity Technology
 * was created by Albert on 12/20/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleOp")
public class teleOp extends Methods {
    Hardware myrobot = new Hardware(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and NavX
        myrobot.init(hardwareMap);

        boolean rotatorPosition = true;
        boolean grabberPosition = true;
        boolean foundationPosition = true;
        ElapsedTime rotatorTimer = new ElapsedTime();
        ElapsedTime grabberTimer = new ElapsedTime();
        ElapsedTime foundationTimer = new ElapsedTime();

        // Adds Telemetry to indicate successful initialization
        telemetry.addData("READY TO PLAY", "!");
        telemetry.addData("Order", Hardware.skyStoneLocation);
        telemetry.update();


        Hardware.targetsSkyStone.deactivate();
        // Driver Press Play
        waitForStart();

        // Main Loop
        while (opModeIsActive()) {
            /*-------------------------------------------------------------------
             *-------------------------------------------------------------------
             * Driving (CALCULATIONS) + Control Inversion
             *-------------------------------------------------------------------
             *-------------------------------------------------------------------*/
            double dRotate_raw = 0;
            double dForward_raw = 0;
            double dStrafe_raw = 0;

            if (Hardware.driver == Hardware.driveOptions.JAVOD) {
                dRotate_raw = Math.pow(gamepad1.left_stick_x, 3);    // Rotate
                dForward_raw = Math.pow(-gamepad1.left_stick_y, 3);  // Forward/Backward
                dStrafe_raw = Math.pow(gamepad1.right_stick_x, 3);   // Strafe

            }
            else if (Hardware.driver == Hardware.driveOptions.BAGUETTE) {
                dRotate_raw = Math.pow(gamepad1.right_stick_x, 3);    // Rotate
                dForward_raw = Math.pow(-gamepad1.left_stick_y, 3);  // Forward/Backward
                dStrafe_raw = Math.pow(gamepad1.left_stick_x, 3);   // Strafe
            }

            if (gamepad1.left_bumper) {
                dForward_raw *= -1;
                dStrafe_raw *= -1;
            }

            // Calculate polar coordinates
            double dMagnitude = Math.hypot(dForward_raw, dStrafe_raw);
            double dAngle = Math.atan2(dForward_raw, dStrafe_raw) - (Math.PI/4);

            // Use deadband to eliminate erroneous joystick values
            if (Math.abs(dMagnitude) < Hardware.dDeadzone) {
                dMagnitude = 0;
            }

            // Calculate power values based on gamepad values, then multiply by sqrt(2)
            double dFrontLeft = (dMagnitude * Math.cos(dAngle) + dRotate_raw) * (Math.sqrt(2));
            double dBackLeft = (dMagnitude * Math.sin(dAngle) + dRotate_raw) * (Math.sqrt(2));
            double dFrontRight = (dMagnitude * Math.sin(dAngle) - dRotate_raw) * (Math.sqrt(2));
            double dBackRight = (dMagnitude * Math.cos(dAngle) - dRotate_raw) * (Math.sqrt(2));

            // Clip Power Values not to exceed -1/+1
            dFrontLeft = Range.clip(dFrontLeft, -1, 1);
            dBackLeft = Range.clip(dBackLeft, -1, 1);
            dFrontRight = Range.clip(dFrontRight, -1, 1);
            dBackRight = Range.clip(dBackRight, -1, 1);

            myrobot.bSlowmode = false;

            // Divide speed by slowing factor

            if (gamepad1.right_bumper) {
                myrobot.bSlowmode = true;
                dFrontLeft = (dFrontLeft / myrobot.dSlowfactor);
                dBackLeft = (dBackLeft / myrobot.dSlowfactor);
                dFrontRight = (dFrontRight / myrobot.dSlowfactor);
                dBackRight = (dBackRight / myrobot.dSlowfactor);
            }

            /*-------------------------------------------------------------------
             *-------------------------------------------------------------------
             *  Other Motors (CALCULATIONS)
             *-------------------------------------------------------------------
             *-------------------------------------------------------------------*/

            //Vertical Extenders
            double verticalExtenderPowerRaw = Math.pow(-gamepad2.left_stick_y, 3);

            //Collector
            double collectorPowerRawForward = Math.pow(-gamepad2.left_trigger, 3);
            double collectorPowerRawBackward = Math.pow(gamepad2.right_trigger, 3);

            //Horizontal Extenders
            double horizontalExtenderPowerRaw = Math.pow(-gamepad2.left_stick_x, 3);
            double horizontalExtenderPower = horizontalExtenderPowerRaw * 0.5;

            /*-------------------------------------------------------------------
             *-------------------------------------------------------------------
             * Motors (IMPLEMENTATIONS)
             *-------------------------------------------------------------------
             *-------------------------------------------------------------------*/
            //Extender

            // Perform driving function
            myrobot.leftFrontMotor.setPower(dFrontLeft);
            myrobot.leftBackMotor.setPower(dBackLeft);
            myrobot.rightFrontMotor.setPower(dFrontRight);
            myrobot.rightBackMotor.setPower(dBackRight);

            myrobot.verticalExtenderLeft.setPower(verticalExtenderPowerRaw);
            myrobot.verticalExtenderRight.setPower(verticalExtenderPowerRaw);

            myrobot.collectorLeft.setPower(collectorPowerRawBackward + collectorPowerRawForward);
            myrobot.collectorRight.setPower(collectorPowerRawBackward + collectorPowerRawForward);

            if (Hardware.side == Hardware.sideOptions.RED) {
                myrobot.horizontalExtenderLeft.setPower(horizontalExtenderPower);
            }

            if (Hardware.side == Hardware.sideOptions.BLUE) {
                myrobot.horizontalExtenderLeft.setPower(-horizontalExtenderPower);
            }

            if (Hardware.side == Hardware.sideOptions.NULL) {
                myrobot.horizontalExtenderLeft.setPower(-horizontalExtenderPower);
            }

            if (gamepad2.a && rotatorPosition && rotatorTimer.seconds() > 1) {
                myrobot.rotator.setPosition(0);
                rotatorPosition = false;
                rotatorTimer.reset();
            }

            if (gamepad2.a && !rotatorPosition && rotatorTimer.seconds() > 1) {
                myrobot.rotator.setPosition(0.95);
                rotatorPosition = true;
                rotatorTimer.reset();
            }

            if (gamepad2.b && grabberPosition && grabberTimer.seconds() > 1) {
                myrobot.grabber.setPosition(0);
                grabberPosition = false;
                grabberTimer.reset();
            }

            if (gamepad2.b && !grabberPosition && grabberTimer.seconds() > 1) {
                myrobot.LED.setPosition(0.6145);
                myrobot.grabber.setPosition(0.4);
                grabberPosition = true;
                grabberTimer.reset();
            }

            if (!grabberPosition && !(myrobot.colorSensor.red() > 5 && myrobot.colorSensor.green() > 5)) {
                myrobot.LED.setPosition(0.2425);
            }

            if (gamepad2.y) {
                myrobot.teamMarker.setPosition(0.1);
            }

            if (gamepad2.x && foundationPosition && foundationTimer.seconds() > 1) {
                if (Hardware.side == Hardware.sideOptions.BLUE) {
                    myrobot.LED.setPosition(0.7445);
                }
                else {
                    myrobot.LED.setPosition(0.6695);
                }
                myrobot.foundationLeft.setPosition(1);
                myrobot.foundationRight.setPosition(0);
                foundationPosition = false;
                foundationTimer.reset();
            }

            if (gamepad2.x && !foundationPosition && foundationTimer.seconds() > 1) {
                myrobot.LED.setPosition(0.6145);
                myrobot.foundationLeft.setPosition(0.1);
                myrobot.foundationRight.setPosition(0.9);
                foundationPosition = true;
                foundationTimer.reset();
            }

            if (myrobot.colorSensor.red() > 5 && myrobot.colorSensor.green() > 5) {
                if (!grabberPosition) {
                    myrobot.LED.setPosition(0.7595);
                }
                else {
                    myrobot.LED.setPosition(0.6745);
                }
                myrobot.pusher.setPosition(0.50);
            }

            else {
                myrobot.pusher.setPosition(0);
            }
        }
    }
}