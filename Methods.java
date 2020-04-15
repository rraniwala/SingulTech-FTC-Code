package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/*
 * This awesome program for Singularity Technology
 * was created by Albert on 2/24/2017.
 */
abstract public class Methods extends LinearOpMode {
    /*
     * Base Methods
     */
    public void matchConfigure(Hardware myrobot){
        ElapsedTime configurationTime = new ElapsedTime();
        while(configurationTime.seconds() < 30) {

            if (isStopRequested()) {
                telemetry.addData("STOPPING...", "CONFIGURATION UNFINISHED");
                telemetry.addData("Warning:", "May result in errors");
                telemetry.update();
                return;
            }

            if (gamepad1.y){
                break;
            }

            if (gamepad2.b) {
                myrobot.LED.setPosition(0.6695);
                Hardware.side = Hardware.sideOptions.RED;
            }

            if (gamepad2.x) {
                myrobot.LED.setPosition(0.7445);
                Hardware.side = Hardware.sideOptions.BLUE;
                Hardware.translateSpeed = 0.7;
            }

            if (gamepad1.x) {
                Hardware.driver = Hardware.driveOptions.JAVOD;
            }

            if (gamepad1.b) {
                Hardware.driver = Hardware.driveOptions.BAGUETTE;
            }

            if (gamepad2.dpad_up) {
                Hardware.autoWaitTime ++;
                Hardware.autoWaitTime = Range.clip(Hardware.autoWaitTime, 0, 5);
                sleep(100);
            }

            if(gamepad2.dpad_down) {
                Hardware.autoWaitTime --;
                Hardware.autoWaitTime = Range.clip(Hardware.autoWaitTime, 0, 5);
                sleep(200);
            }

            telemetry.addData("Side", Hardware.side);
            telemetry.addData("Driver", Hardware.driver);
            telemetry.update();
        }
        telemetry.addData("Configuration is complete", "!");
        telemetry.addData("Side", Hardware.side);
        telemetry.addData("Driver", Hardware.driver);
        telemetry.addData("Delay", Hardware.autoWaitTime);
        telemetry.update();
    }

    /*
    public double adjustedDistance(Hardware myrobot) {
        return myrobot.distanceSensor.getDistance(DistanceUnit.INCH) * 1.1 - 0.52;
    }
    */

    /*
    Drive Methods
     */
    public void drive(Hardware myrobot, double distance, double Speed, Hardware.driveDirection direction) {
        double LFspeed;
        double RFspeed;
        double LBspeed;
        double RBspeed;
        boolean movement = true;


        if (direction == Hardware.driveDirection.RIGHT || direction == Hardware.driveDirection.LEFT
                || direction == Hardware.driveDirection.LEFT_BACKWARD || direction == Hardware.driveDirection.RIGHT_BACKWARD ||
                direction == Hardware.driveDirection.LEFT_FORWARD || direction == Hardware.driveDirection.RIGHT_FORWARD) {
            distance *= Math.sqrt(2);
        }

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            int newLeftBackTarget = 0;
            int newRightBackTarget = 0;
            int newLeftFrontTarget = 0;
            int newRightFrontTarget = 0;

            // Forwards motion
            if (direction.equals(Hardware.driveDirection.FORWARD)) {
                newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
                myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
                myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            }

            // Backwards motion
            if (direction.equals(Hardware.driveDirection.BACKWARD)) {
                newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
                myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
                myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            }

            // Strafe motions
            if (direction.equals(Hardware.driveDirection.RIGHT)) {
                newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
                myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
                myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            }
            if (direction.equals(Hardware.driveDirection.LEFT)) {
                newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
                myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
                myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            }

            // Diagonal motions
            if (direction.equals(Hardware.driveDirection.RIGHT_FORWARD)) {
                newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);

                myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
                myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                myrobot.leftBackMotor.setTargetPosition(myrobot.leftBackMotor.getCurrentPosition());
                myrobot.rightFrontMotor.setTargetPosition(myrobot.rightFrontMotor.getCurrentPosition());
            }

            if (direction.equals(Hardware.driveDirection.LEFT_BACKWARD)) {
                newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);

                myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
                myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                myrobot.leftBackMotor.setTargetPosition(myrobot.leftBackMotor.getCurrentPosition());
                myrobot.rightFrontMotor.setTargetPosition(myrobot.rightFrontMotor.getCurrentPosition());
            }

            if (direction.equals(Hardware.driveDirection.LEFT_FORWARD)) {
                newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() + (int) (distance * Hardware.COUNTS_PER_INCH);

                myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
                myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
                myrobot.leftFrontMotor.setTargetPosition(myrobot.leftFrontMotor.getCurrentPosition());
                myrobot.rightBackMotor.setTargetPosition(myrobot.rightBackMotor.getCurrentPosition());
            }

            if (direction.equals(Hardware.driveDirection.RIGHT_BACKWARD)) {
                newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);
                newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() - (int) (distance * Hardware.COUNTS_PER_INCH);

                myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
                myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
                myrobot.leftFrontMotor.setTargetPosition(myrobot.leftFrontMotor.getCurrentPosition());
                myrobot.rightBackMotor.setTargetPosition(myrobot.rightBackMotor.getCurrentPosition());
            }
            // Take angle for speed based on direction
            double NewAngle = 0;

            if (direction.equals(Hardware.driveDirection.FORWARD)) {
                NewAngle = (Math.PI / 4);
            }
            if (direction.equals(Hardware.driveDirection.BACKWARD)) {
                NewAngle = -(3 * Math.PI / 4);
            }
            if (direction.equals(Hardware.driveDirection.LEFT)) {
                NewAngle = -(Math.PI / 4);
            }
            if (direction.equals(Hardware.driveDirection.RIGHT)) {
                NewAngle = (3 * Math.PI / 4);
            }
            if (direction.equals(Hardware.driveDirection.LEFT_FORWARD)) {
                NewAngle = 0;
            }
            if (direction.equals(Hardware.driveDirection.LEFT_BACKWARD)) {
                NewAngle = -(Math.PI / 2);
            }
            if (direction.equals(Hardware.driveDirection.RIGHT_FORWARD)) {
                NewAngle = (Math.PI / 2);
            }
            if (direction.equals(Hardware.driveDirection.RIGHT_BACKWARD)) {
                NewAngle = Math.PI;
            }


            // Turn On RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Reset the timeout time and start motion.
            LBspeed = Speed * Math.cos(NewAngle) * Math.sqrt(2);
            RBspeed = Speed * Math.sin(NewAngle) * Math.sqrt(2);
            LFspeed = Speed * Math.sin(NewAngle) * Math.sqrt(2);
            RFspeed = Speed * Math.cos(NewAngle) * Math.sqrt(2);

            myrobot.leftFrontMotor.setPower(LFspeed);
            myrobot.rightFrontMotor.setPower(RFspeed);
            myrobot.leftBackMotor.setPower(LBspeed);
            myrobot.rightBackMotor.setPower(RBspeed);

            myrobot.runtime.reset();

            // Keep looping while there is time left, and all motors are running.
            while ((opModeIsActive()) && (myrobot.runtime.seconds() < 10) && (movement)) {
                //LFspeed = Speed * Math.sin(NewAngle) * Math.sqrt(2) * deacceleration(myrobot, newLeftFrontTarget, newRightFrontTarget, Speed);
                //RFspeed = Speed * Math.cos(NewAngle) * Math.sqrt(2) * deacceleration(myrobot, newLeftFrontTarget, newRightFrontTarget, Speed);
                //LBspeed = Speed * Math.cos(NewAngle) * Math.sqrt(2) * deacceleration(myrobot, newLeftFrontTarget, newRightFrontTarget, Speed);
                //RBspeed = Speed * Math.sin(NewAngle) * Math.sqrt(2) * deacceleration(myrobot, newLeftFrontTarget, newRightFrontTarget, Speed);

                myrobot.leftFrontMotor.setPower(LFspeed);
                myrobot.rightFrontMotor.setPower(RFspeed);
                myrobot.leftBackMotor.setPower(LBspeed);
                myrobot.rightBackMotor.setPower(RBspeed);


                movement = (Math.abs(myrobot.leftBackMotor.getCurrentPosition() - myrobot.leftBackMotor.getTargetPosition()) > 5) &&
                        (Math.abs(myrobot.rightBackMotor.getCurrentPosition() - myrobot.rightBackMotor.getTargetPosition()) > 5) &&
                        (Math.abs(myrobot.leftFrontMotor.getCurrentPosition() - myrobot.leftFrontMotor.getTargetPosition()) > 5) &&
                        (Math.abs(myrobot.rightFrontMotor.getCurrentPosition() - myrobot.rightFrontMotor.getTargetPosition()) > 5);


                if (direction.equals(Hardware.driveDirection.LEFT_FORWARD) || direction.equals(Hardware.driveDirection.RIGHT_BACKWARD)) {
                    movement = (Math.abs(myrobot.leftBackMotor.getCurrentPosition() - myrobot.leftBackMotor.getTargetPosition()) > 5) &&
                            (Math.abs(myrobot.rightFrontMotor.getCurrentPosition() - myrobot.rightFrontMotor.getTargetPosition()) > 5);
                }
                if (direction.equals(Hardware.driveDirection.LEFT_BACKWARD) || direction.equals(Hardware.driveDirection.RIGHT_FORWARD)) {
                    movement = (Math.abs(myrobot.leftFrontMotor.getCurrentPosition() - myrobot.leftFrontMotor.getTargetPosition()) > 5) &&
                            (Math.abs(myrobot.rightBackMotor.getCurrentPosition() - myrobot.rightBackMotor.getTargetPosition()) > 5);
                }



                telemetry.addData("Remainder", "%4d :%4d :%4d :%4d",
                        newLeftFrontTarget - myrobot.leftFrontMotor.getCurrentPosition(),
                        newRightFrontTarget - myrobot.rightFrontMotor.getCurrentPosition(),
                        newLeftBackTarget - myrobot.leftBackMotor.getCurrentPosition(),
                        newRightBackTarget - myrobot.rightBackMotor.getCurrentPosition());

                telemetry.addData("Power", "%4f :%4f :%4f :%4f",
                        myrobot.leftFrontMotor.getPower(),
                        myrobot.rightFrontMotor.getPower(),
                        myrobot.leftBackMotor.getPower(),
                        myrobot.rightBackMotor.getPower());

                telemetry.addData("Current Position", "%4d :%4d :%4d :%4d",
                        myrobot.leftFrontMotor.getCurrentPosition(),
                        myrobot.rightFrontMotor.getCurrentPosition(),
                        myrobot.leftBackMotor.getCurrentPosition(),
                        myrobot.rightBackMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            myrobot.rightFrontMotor.setPower(0);
            myrobot.leftBackMotor.setPower(0);
            myrobot.rightBackMotor.setPower(0);
            myrobot.leftFrontMotor.setPower(0);

            telemetry.addData(">", "Stopping motion");
            telemetry.update();

            // Turn off RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        /*
        if (direction == Hardware.driveDirection.LEFT || direction == Hardware.driveDirection.RIGHT
                || direction == Hardware.driveDirection.LEFT_BACKWARD || direction == Hardware.driveDirection.RIGHT_BACKWARD) {
            encoderTurn(myrobot, 0, Hardware.turnSpeed);
        }
        */
    }

    public void encoderTurn (Hardware myrobot, double degrees, double Speed) {
        if ((myrobot.IMULeft.getAngularOrientation().firstAngle > 170 && myrobot.IMURight.getAngularOrientation().firstAngle < -170) ||
                (myrobot.IMULeft.getAngularOrientation().firstAngle < -170 && myrobot.IMURight.getAngularOrientation().firstAngle > 170)) {
            myrobot.currentAngle = (360 + myrobot.IMULeft.getAngularOrientation().firstAngle + myrobot.IMURight.getAngularOrientation().firstAngle) / 2;
        }
        else {
            myrobot.currentAngle = (myrobot.IMULeft.getAngularOrientation().firstAngle + myrobot.IMURight.getAngularOrientation().firstAngle) / 2;
        }

        double error;

        if (Hardware.expectedAngle > 170 && myrobot.currentAngle < -170) {
            error = Hardware.expectedAngle - (360 + myrobot.currentAngle);
        }
        else if (Hardware.expectedAngle < -170 && myrobot.currentAngle > 170) {
            error = Hardware.expectedAngle - (myrobot.currentAngle - 360);
        }
        else {
            error = Hardware.expectedAngle - myrobot.currentAngle;
        }

        double realDegrees = degrees + (error);

        Hardware.expectedAngle += degrees;
        while (Hardware.expectedAngle < -180) {
            Hardware.expectedAngle += 360;
        }
        while (Hardware.expectedAngle > 180) {
            Hardware.expectedAngle -= 360;
        }
        double LFspeed;
        double RFspeed;
        double LBspeed;
        double RBspeed;
        boolean movement = true;

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            int newLeftBackTarget;
            int newRightBackTarget;
            int newLeftFrontTarget;
            int newRightFrontTarget;

            // Sets motion
            newLeftBackTarget = myrobot.leftBackMotor.getCurrentPosition() - (int) (Hardware.INCHES_PER_DEGREE * realDegrees * Hardware.REAL_DEGREE_COEFF * Hardware.COUNTS_PER_INCH);
            newRightBackTarget = myrobot.rightBackMotor.getCurrentPosition() + (int) (Hardware.INCHES_PER_DEGREE * realDegrees * Hardware.REAL_DEGREE_COEFF * Hardware.COUNTS_PER_INCH);
            newLeftFrontTarget = myrobot.leftFrontMotor.getCurrentPosition() - (int) (Hardware.INCHES_PER_DEGREE * realDegrees * Hardware.REAL_DEGREE_COEFF * Hardware.COUNTS_PER_INCH);
            newRightFrontTarget = myrobot.rightFrontMotor.getCurrentPosition() + (int) (Hardware.INCHES_PER_DEGREE * realDegrees * Hardware.REAL_DEGREE_COEFF * Hardware.COUNTS_PER_INCH);
            myrobot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            myrobot.rightBackMotor.setTargetPosition(newRightBackTarget);
            myrobot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            myrobot.rightFrontMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Reset the timeout time and start motion.
            LBspeed = -Speed;
            RBspeed = Speed;
            LFspeed = -Speed;
            RFspeed = Speed;

            myrobot.leftFrontMotor.setPower(LFspeed);
            myrobot.rightFrontMotor.setPower(RFspeed);
            myrobot.leftBackMotor.setPower(LBspeed);
            myrobot.rightBackMotor.setPower(RBspeed);

            myrobot.runtime.reset();

            // Keep looping while there is time left, and all motors are running.
            while ((opModeIsActive()) && (myrobot.runtime.seconds() < 30) && (movement)) {

                myrobot.leftFrontMotor.setPower(LFspeed);
                myrobot.rightFrontMotor.setPower(RFspeed);
                myrobot.leftBackMotor.setPower(LBspeed);
                myrobot.rightBackMotor.setPower(RBspeed);


                movement = (Math.abs(myrobot.leftBackMotor.getCurrentPosition() - myrobot.leftBackMotor.getTargetPosition()) > 5) &&
                        (Math.abs(myrobot.rightBackMotor.getCurrentPosition() - myrobot.rightBackMotor.getTargetPosition()) > 5) &&
                        (Math.abs(myrobot.leftFrontMotor.getCurrentPosition() - myrobot.leftFrontMotor.getTargetPosition()) > 5) &&
                        (Math.abs(myrobot.rightFrontMotor.getCurrentPosition() - myrobot.rightFrontMotor.getTargetPosition()) > 5);



                //movement = (myrobot.leftBackMotor.isBusy() && myrobot.rightFrontMotor.isBusy() && myrobot.leftFrontMotor.isBusy() && myrobot.rightBackMotor.isBusy());


                //telemetry.update();
                myrobot.currentAngle = myrobot.IMULeft.getAngularOrientation().firstAngle;
                telemetry.addData("Expected Angle", Hardware.expectedAngle);
                telemetry.addData("Current Angle", myrobot.currentAngle);
                telemetry.addData("Error", Hardware.expectedAngle - myrobot.currentAngle);
                System.out.println("Expected Angle: " + Hardware.expectedAngle + " Current Angle: " + myrobot.currentAngle + " Error: " + (Hardware.expectedAngle - myrobot.currentAngle));
                telemetry.update();
            }

            if ((myrobot.IMULeft.getAngularOrientation().firstAngle > 170 && myrobot.IMURight.getAngularOrientation().firstAngle < -170) ||
                    (myrobot.IMULeft.getAngularOrientation().firstAngle < -170 && myrobot.IMURight.getAngularOrientation().firstAngle > 170)) {
                myrobot.currentAngle = (360 + myrobot.IMULeft.getAngularOrientation().firstAngle + myrobot.IMURight.getAngularOrientation().firstAngle) / 2;
            }
            else {
                myrobot.currentAngle = (myrobot.IMULeft.getAngularOrientation().firstAngle + myrobot.IMURight.getAngularOrientation().firstAngle) / 2;
            }

            if (Hardware.expectedAngle > 170 && myrobot.currentAngle < -170) {
                error = Hardware.expectedAngle - (360 + myrobot.currentAngle);
            }
            else if (Hardware.expectedAngle < -170 && myrobot.currentAngle > 170) {
                error = Hardware.expectedAngle - (myrobot.currentAngle - 360);
            }
            else {
                error = Hardware.expectedAngle - myrobot.currentAngle;
            }

            if (Math.abs(error) > 1.5) {
                telemetry.addData("New", "Turn");
                System.out.println("New Turn");
                telemetry.update();
                encoderTurn(myrobot, 0, Hardware.turnSpeed);
            }



            // Stop all motion;
            myrobot.leftBackMotor.setPower(0);
            myrobot.rightBackMotor.setPower(0);
            myrobot.leftFrontMotor.setPower(0);
            myrobot.rightFrontMotor.setPower(0);

            telemetry.addData(">", "Stopping motion");
            telemetry.update();

            // Turn off RUN_TO_POSITION
            myrobot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            myrobot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void powerDrive(Hardware myrobot, double time, double Speed, Hardware.driveDirection direction) {
        double LFspeed;
        double RFspeed;
        double LBspeed;
        double RBspeed;

        // Take angle for speed based on direction
        double NewAngle = 0;

        if (direction.equals(Hardware.driveDirection.FORWARD)) {
            NewAngle = (Math.PI / 4);
        }
        if (direction.equals(Hardware.driveDirection.BACKWARD)) {
            NewAngle = -(3 * Math.PI / 4);
        }
        if (direction.equals(Hardware.driveDirection.LEFT)) {
            NewAngle = -(Math.PI / 4);
        }
        if (direction.equals(Hardware.driveDirection.RIGHT)) {
            NewAngle = (3 * Math.PI / 4);
        }
        if (direction.equals(Hardware.driveDirection.LEFT_FORWARD)) {
            NewAngle = (Math.PI / 2);
        }
        if (direction.equals(Hardware.driveDirection.LEFT_BACKWARD)) {
            NewAngle = Math.PI;
        }
        if (direction.equals(Hardware.driveDirection.RIGHT_FORWARD)) {
            NewAngle = 0;
        }
        if (direction.equals(Hardware.driveDirection.RIGHT_BACKWARD)) {
            NewAngle = -(Math.PI / 2);
        }


        // Reset the timeout time and start motion.
        LBspeed = Speed * Math.cos(NewAngle) * Math.sqrt(2);
        RBspeed = Speed * Math.sin(NewAngle) * Math.sqrt(2);
        LFspeed = Speed * Math.sin(NewAngle) * Math.sqrt(2);
        RFspeed = Speed * Math.cos(NewAngle) * Math.sqrt(2);

        ElapsedTime movement = new ElapsedTime();
        while (movement.seconds() < time) {
            myrobot.leftFrontMotor.setPower(LFspeed);
            myrobot.rightFrontMotor.setPower(RFspeed);
            myrobot.leftBackMotor.setPower(LBspeed);
            myrobot.rightBackMotor.setPower(RBspeed);
        }

        myrobot.leftFrontMotor.setPower(0);
        myrobot.rightFrontMotor.setPower(0);
        myrobot.leftBackMotor.setPower(0);
        myrobot.rightBackMotor.setPower(0);
    }

    /*
    public void updateOdometry(Hardware myrobot) {
        Thread positionThread = new Thread(myrobot.globalPositionUpdate);
        positionThread.start();
    }
    */

    /*
    Game Specific Methods
    */

    public void startCollectors(Hardware myrobot) {
        myrobot.collectorLeft.setPower(1);
        myrobot.collectorRight.setPower(1);
    }

    public void stopCollectors(Hardware myrobot) {
        myrobot.collectorLeft.setPower(0);
        myrobot.collectorRight.setPower(0);
    }

    public void driveToSkyStone(Hardware myrobot) {
            if (Hardware.side == Hardware.sideOptions.RED) {
                drive(myrobot, 17, Hardware.translateSpeed, Hardware.driveDirection.LEFT);
                drive(myrobot, 4, Hardware.driveSpeed, Hardware.driveDirection.BACKWARD);
            }
            if (Hardware.side == Hardware.sideOptions.BLUE) {
                drive(myrobot, 17.5, Hardware.translateSpeed, Hardware.driveDirection.LEFT);
                drive(myrobot, 14, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
            }
    }

    public Hardware.SkyStoneOrder findSkyStone() {
        Hardware.targetsSkyStone.activate();
        ElapsedTime SkyStoneTimer = new ElapsedTime();
        SkyStoneTimer.reset();
        // check all the trackable targets to see which one (if any) is visible.
        while (SkyStoneTimer.seconds() < 2) {
            telemetry.addData("Searching", "");
            telemetry.update();
            Hardware.targetVisible = false;
            if (((VuforiaTrackableDefaultListener) Hardware.stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", Hardware.stoneTarget.getName());
                Hardware.targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) Hardware.stoneTarget.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    Hardware.lastLocation = robotLocationTransform;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (Hardware.targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = Hardware.lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / Hardware.mmPerInch, translation.get(1) / Hardware.mmPerInch, translation.get(2) / Hardware.mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(Hardware.lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                if (translation.get(1) < 0) {
                    telemetry.addData("One", "Four");
                    telemetry.update();
                    if (Hardware.side == Hardware.sideOptions.RED) {
                        return Hardware.SkyStoneOrder.THREE_SIX;
                    }
                    else {
                        return Hardware.SkyStoneOrder.ONE_FOUR;
                    }
                }
                if (translation.get(1) > 0) {
                    telemetry.addData("Three", "Six");
                    telemetry.update();
                    if (Hardware.side == Hardware.sideOptions.RED) {
                        return Hardware.SkyStoneOrder.ONE_FOUR;
                    }
                    else {
                        return Hardware.SkyStoneOrder.THREE_SIX;
                    }

                }
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
        // Disable Tracking when we are done;
        Hardware.targetsSkyStone.deactivate();
        telemetry.addData("Two", "Five");
        telemetry.update();
        return Hardware.SkyStoneOrder.TWO_FIVE;
    }

    public void skyStonePath (Hardware myrobot) {
        if (Hardware.side == Hardware.sideOptions.RED) {
            if (Hardware.skyStoneLocation == Hardware.SkyStoneOrder.ONE_FOUR) {
                drive(myrobot, 10.75, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
                drive(myrobot, 16, Hardware.translateSpeed, Hardware.driveDirection.LEFT);
                CollectSkyStone collectSkyStone = new CollectSkyStone(myrobot);
                collectSkyStone.start();
                drive(myrobot, 6, Hardware.slowDrive, Hardware.driveDirection.BACKWARD);
                try {
                    collectSkyStone.join();
                } catch (Exception e) {
                    telemetry.addData("Unable to execute thread", "RIP");
                    telemetry.update();
                }
                drive(myrobot, 14, Hardware.translateSpeed, Hardware.driveDirection.RIGHT);
                drive(myrobot, 86, Hardware.fastDrive, Hardware.driveDirection.FORWARD);
            }

            if (Hardware.skyStoneLocation == Hardware.SkyStoneOrder.TWO_FIVE) {
                drive(myrobot, 18, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
                drive(myrobot, 16, Hardware.translateSpeed, Hardware.driveDirection.LEFT);
                CollectSkyStone collectSkyStone = new CollectSkyStone(myrobot);
                collectSkyStone.start();
                drive(myrobot, 6, Hardware.slowDrive, Hardware.driveDirection.BACKWARD);
                try {
                    collectSkyStone.join();
                } catch (Exception e) {
                    telemetry.addData("Unable to execute thread", "RIP");
                    telemetry.update();
                }
                drive(myrobot, 14, Hardware.translateSpeed, Hardware.driveDirection.RIGHT);
                drive(myrobot, 77 , Hardware.fastDrive, Hardware.driveDirection.FORWARD);
            }

            if (Hardware.skyStoneLocation == Hardware.SkyStoneOrder.THREE_SIX) {
                drive(myrobot, 3.25, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
                drive(myrobot, 16, Hardware.translateSpeed, Hardware.driveDirection.LEFT);
                CollectSkyStone collectSkyStone = new CollectSkyStone(myrobot);
                collectSkyStone.start();
                drive(myrobot, 6, Hardware.slowDrive, Hardware.driveDirection.BACKWARD);
                try {
                    collectSkyStone.join();
                } catch (Exception e) {
                    telemetry.addData("Unable to execute thread", "RIP");
                    telemetry.update();
                }
                drive(myrobot, 14, Hardware.translateSpeed, Hardware.driveDirection.RIGHT);
                drive(myrobot, 94, Hardware.fastDrive, Hardware.driveDirection.FORWARD);
            }
        }

        else {
            if (Hardware.skyStoneLocation == Hardware.SkyStoneOrder.ONE_FOUR) {
                drive(myrobot, 2.5, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
                drive(myrobot, 16.5, Hardware.translateSpeed, Hardware.driveDirection.LEFT);
                CollectSkyStone collectSkyStone = new CollectSkyStone(myrobot);
                collectSkyStone.start();
                drive(myrobot, 6, Hardware.slowDrive, Hardware.driveDirection.BACKWARD);
                try {
                    collectSkyStone.join();
                } catch (Exception e) {
                    telemetry.addData("Unable to execute thread", "RIP");
                    telemetry.update();
                }
                drive(myrobot, 12, Hardware.translateSpeed, Hardware.driveDirection.RIGHT);
                drive(myrobot, 91, Hardware.fastDrive, Hardware.driveDirection.BACKWARD);
            }

            if (Hardware.skyStoneLocation == Hardware.SkyStoneOrder.TWO_FIVE) {
                drive(myrobot, 5, Hardware.driveSpeed, Hardware.driveDirection.BACKWARD);
                drive(myrobot, 16.5, Hardware.translateSpeed, Hardware.driveDirection.LEFT);
                CollectSkyStone collectSkyStone = new CollectSkyStone(myrobot);
                collectSkyStone.start();
                drive(myrobot, 6, Hardware.slowDrive, Hardware.driveDirection.BACKWARD);
                try {
                    collectSkyStone.join();
                } catch (Exception e) {
                    telemetry.addData("Unable to execute thread", "RIP");
                    telemetry.update();
                }
                drive(myrobot, 12, Hardware.translateSpeed, Hardware.driveDirection.RIGHT);
                drive(myrobot, 83, Hardware.fastDrive, Hardware.driveDirection.BACKWARD);
            }

            if (Hardware.skyStoneLocation == Hardware.SkyStoneOrder.THREE_SIX) {
                drive(myrobot, 12.5, Hardware.driveSpeed, Hardware.driveDirection.BACKWARD);
                drive(myrobot, 16.5, Hardware.translateSpeed, Hardware.driveDirection.LEFT);
                CollectSkyStone collectSkyStone = new CollectSkyStone(myrobot);
                collectSkyStone.start();
                drive(myrobot, 6, Hardware.slowDrive, Hardware.driveDirection.BACKWARD);
                try {
                    collectSkyStone.join();
                } catch (Exception e) {
                    telemetry.addData("Unable to execute thread", "RIP");
                    telemetry.update();
                }
                drive(myrobot, 12, Hardware.translateSpeed, Hardware.driveDirection.RIGHT);
                drive(myrobot, 75, Hardware.fastDrive, Hardware.driveDirection.BACKWARD);
            }
        }
    }

    public void moveFoundation(Hardware myrobot) {
        if (Hardware.side == Hardware.sideOptions.RED) {
            encoderTurn(myrobot, 90, Hardware.turnSpeed);
            PositionSkyStone positionSkyStone = new PositionSkyStone(myrobot);
            positionSkyStone.start();
            drive(myrobot, 7, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
            encoderTurn(myrobot, 0, Hardware.turnSpeed);
            drive(myrobot, 7, Hardware.slowDrive, Hardware.driveDirection.FORWARD);
            encoderTurn(myrobot, 0, Hardware.turnSpeed);
            dropFoundation(myrobot);
            drive(myrobot, 34, Hardware.driveSpeed, Hardware.driveDirection.BACKWARD);
            DepositSkyStone depositSkyStone = new DepositSkyStone(myrobot);
            depositSkyStone.start();
            drive(myrobot, 4, Hardware.slowDrive, Hardware.driveDirection.BACKWARD);
            raiseFoundation(myrobot);
            encoderTurn(myrobot, 0, Hardware.turnSpeed);
            myrobot.LED.setPosition(0.7145);
            if(!(myrobot.colorSensor.red() > 5 && myrobot.colorSensor.green() > 5)) {
                drive(myrobot, 20, Hardware.translateSpeed, Hardware.driveDirection.LEFT);
                drive(myrobot, 30, Hardware.translateSpeed, Hardware.driveDirection.LEFT_FORWARD);
            }
            myrobot.LED.setPosition(0.6445);
        }
        else {
            encoderTurn(myrobot, 90, Hardware.turnSpeed);
            PositionSkyStone positionSkyStone = new PositionSkyStone(myrobot);
            positionSkyStone.start();
            drive(myrobot, 7, Hardware.driveSpeed, Hardware.driveDirection.FORWARD);
            encoderTurn(myrobot, 0, Hardware.turnSpeed);
            drive(myrobot, 7, Hardware.slowDrive, Hardware.driveDirection.FORWARD);
            encoderTurn(myrobot, 0, Hardware.turnSpeed);
            dropFoundation(myrobot);
            drive(myrobot, 34, Hardware.driveSpeed, Hardware.driveDirection.BACKWARD);
            DepositSkyStone depositSkyStone = new DepositSkyStone(myrobot);
            depositSkyStone.start();
            drive(myrobot, 4, Hardware.slowDrive, Hardware.driveDirection.BACKWARD);
            encoderTurn(myrobot, 0, Hardware.turnSpeed);
            raiseFoundation(myrobot);

        }
    }

    public void raiseFoundation (Hardware myrobot) {
        sleep(500);
        myrobot.foundationLeft.setPosition(0.1);
        myrobot.foundationRight.setPosition(0.9);
        sleep(500);
    }

    public void dropFoundation (Hardware myrobot) {
        sleep(500);
        myrobot.foundationLeft.setPosition(0.9);
        myrobot.foundationRight.setPosition(0.1);
        sleep(500);
    }

    class CollectSkyStone extends Thread {
        Hardware myrobot;
        public CollectSkyStone(Hardware myrobot) {
            this.myrobot = myrobot;
        }

        public void run() {
            ElapsedTime collection = new ElapsedTime();
            startCollectors(myrobot);
            while (collection.seconds() < 3) {
                if (myrobot.colorSensor.red() > 5 && myrobot.colorSensor.green() > 5) {
                    myrobot.pusher.setPosition(0.50);
                    myrobot.LED.setPosition(0.6945);
                    break;
                }
            }
            myrobot.grabber.setPosition(0);
            stopCollectors(myrobot);
        }
    }

    class PositionSkyStone extends Thread {
        Hardware myrobot;
        public PositionSkyStone(Hardware myrobot) {
            this.myrobot = myrobot;
        }

        public void run() {
            ElapsedTime position = new ElapsedTime();

            while (position.seconds() < 0.55) {
                myrobot.verticalExtenderRight.setPower(0.7);
                myrobot.verticalExtenderLeft.setPower(0.7);
                if (myrobot.colorSensor.red() < 5 && myrobot.colorSensor.green() < 5) {
                    myrobot.pusher.setPosition(0);
                }
            }
            myrobot.verticalExtenderRight.setPower(0);
            myrobot.verticalExtenderLeft.setPower(0);

            position.reset();
            while (position.seconds() < 1.5) {
                myrobot.horizontalExtenderLeft.setPower(-0.5);
            }
            myrobot.horizontalExtenderLeft.setPower(0);

            myrobot.rotator.setPosition(0);
        }
    }

    class DepositSkyStone extends Thread {
        Hardware myrobot;

        public DepositSkyStone(Hardware myrobot) {
            this.myrobot = myrobot;
        }

        public void run() {
            myrobot.grabber.setPosition(0.4);

            ElapsedTime deposit = new ElapsedTime();
            ElapsedTime rotate = new ElapsedTime();
            while (deposit.seconds() < 1.5) {
                if (rotate.seconds() > 0.2) {
                    myrobot.rotator.setPosition(0.95);
                }
                myrobot.horizontalExtenderLeft.setPower(0.5);
            }
            myrobot.horizontalExtenderLeft.setPower(0);

            deposit.reset();
            while (deposit.seconds() < 0.55) {
                myrobot.verticalExtenderRight.setPower(-0.7);
                myrobot.verticalExtenderLeft.setPower(-0.7);
            }
            myrobot.verticalExtenderRight.setPower(0);
            myrobot.verticalExtenderLeft.setPower(0);
        }
    }
}

