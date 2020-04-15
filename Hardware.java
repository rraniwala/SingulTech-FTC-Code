package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/*
 * This awesome program for Singularity Technology
 * was created by Albert on 12/20/2016.
 */

public class Hardware {
    /*------------------------------------------------------------------
     *-------------------------------------------------------------------
     * Hardware Initializations
     *-------------------------------------------------------------------
     *-----------------------------------------------------------------*/
    //Motors
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;

    public DcMotor verticalExtenderLeft;
    public DcMotor verticalExtenderRight;

    public CRServo horizontalExtenderLeft;

    public DcMotor collectorLeft;
    public DcMotor collectorRight;

    public Servo rotator;
    public Servo grabber;
    public Servo pusher;
    public Servo teamMarker;

    public Servo foundationLeft;
    public Servo foundationRight;

    public Servo LED;

    public ModernRoboticsI2cColorSensor colorSensor;

    public enum driveDirection{
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
        LEFT_FORWARD,
        RIGHT_FORWARD,
        LEFT_BACKWARD,
        RIGHT_BACKWARD
    }

    //Onboard IMU
    public BNO055IMU IMULeft;
    public BNO055IMU IMURight;
    public double currentAngle = 0;
    public static double expectedAngle = 0;

    //Sensors
    public enum SkyStoneOrder {
        ONE_FOUR,
        TWO_FIVE,
        THREE_SIX
    }

    public enum sideOptions {
        RED,
        BLUE,
        NULL
    }

    public enum driveOptions {
        JAVOD,
        BAGUETTE
    }

    public static driveOptions driver = driveOptions.JAVOD;
    public static sideOptions side = sideOptions.NULL;
    public static SkyStoneOrder skyStoneLocation = SkyStoneOrder.TWO_FIVE;

    //Vuforia Variables
    public static final String VUFORIA_KEY = "AZWE6Gv/////AAABmcCIlG7TDU0MtjBZBiMd5rcdHvJ7NH8SgVRm4bj3WVPV8u+q9unBuyaJbjM9KE7HtPwXxDe0qhe/1ZQiwTPtGUdMPPsg6PixdPiBfDNP3VYxJa9bDQC+JKoAtVzM68O/VoEzXokXGjXJY2nVH1TeC1qzZlPoZPFjl5pyaHOqFj2Fyu5NykQBTAD7VfatD9nE53KCTIJ+VN8MTtgw9HjgGLIyBcYmkeqhCGiTSuxtxLC57yu9izwvmpg8LZnBtimbvuNs2hKYboGzt9LTQztUeTbTnkTB6bPSAZ60ChdXLObsTi3FsLy3z3eLrWJ3g9256nvq5OV1n0TSb1dh7pP3lecOskQ3c8K/PNZ5sK3sZOBE";

    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false;

    public static VuforiaLocalizer vuforia;
    public static VuforiaLocalizer.Parameters parameters;

    public static final float mmPerInch = 25.4f;

    public static final float stoneZ = 2.00f * mmPerInch;

    public static OpenGLMatrix lastLocation = null;
    public static boolean targetVisible = false;
    public static float phoneXRotate    = 0;
    public static float phoneYRotate    = 0;
    public static float phoneZRotate    = 0;


    public static VuforiaTrackables targetsSkyStone;

    public static VuforiaTrackable stoneTarget;

    // Variables for drive
    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 0.5;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double INCHES_PER_DEGREE = 0.1513;
    static final double REAL_DEGREE_COEFF = 1.558;

    public static double driveSpeed = 0.7;
    public static double turnSpeed = 0.5;
    public static double translateSpeed = 0.5;
    public static double fastDrive = 1;
    public static double slowDrive = 0.3;
    public static double autoWaitTime = 0;


    boolean bSlowmode;
    double dSlowfactor = 4;

    // Variable for a threshold (deadband) to eliminate erroneous gamepad values
    final static double dDeadzone = 0.02;

    //Odometry
    /*
    public DcMotor verticalOdometryRight;
    public DcMotor verticalOdometryLeft;
    public DcMotor horizontalOdometry;

    final double ODOMETRY_COUNTS_PER_INCH = 307.699557;

    public OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalOdometryLeft, verticalOdometryRight, horizontalOdometry, ODOMETRY_COUNTS_PER_INCH, 50);
    */

    /*------------------------------------------------------------------
     *-------------------------------------------------------------------
     * Initialization
     *-------------------------------------------------------------------
     *-----------------------------------------------------------------*/

    // Initializes hardware
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    // Initializes telemetry
    Telemetry mytelemetry = null;

    public Hardware(Telemetry telemetry) {
        this.mytelemetry = telemetry;
    }

    // INITIALIZATION METHOD
    public void init(HardwareMap ahwMap) throws InterruptedException {
        hwMap = ahwMap;
        LED = hwMap.servo.get("LED");
        LED.setPosition(0.6695);
        expectedAngle = 0;
        mytelemetry.addData("Hardware Map", "Initializing");
        mytelemetry.update();
        
        // Get references from hardware map
        leftFrontMotor = hwMap.dcMotor.get("leftFront");
        leftBackMotor = hwMap.dcMotor.get("leftBack");
        rightFrontMotor = hwMap.dcMotor.get("rightFront");
        rightBackMotor = hwMap.dcMotor.get("rightBack");

        verticalExtenderLeft = hwMap.dcMotor.get("verticalExtenderLeft");
        verticalExtenderRight = hwMap.dcMotor.get("verticalExtenderRight");

        collectorLeft = hwMap.dcMotor.get("collectorLeft");
        collectorRight = hwMap.dcMotor.get("collectorRight");

        horizontalExtenderLeft = hwMap.crservo.get("horizontalExtenderLeft");

        rotator = hwMap.servo.get("rotator");
        grabber = hwMap.servo.get("grabber");
        pusher = hwMap.servo.get("pusher");
        teamMarker = hwMap.servo.get("teamMarker");

        foundationLeft = hwMap.servo.get("foundationLeft");
        foundationRight = hwMap.servo.get("foundationRight");

        colorSensor = hwMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");
        IMULeft = hwMap.get(BNO055IMU.class, "IMULeft");
        IMURight = hwMap.get(BNO055IMU.class, "IMURight");

        mytelemetry.addData("Hardware Map", "Complete");
        mytelemetry.addData("DC Motors", "Initializating");
        mytelemetry.update();

        // Reset Encoders to 0
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalExtenderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalExtenderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse motors
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        verticalExtenderLeft.setDirection(DcMotor.Direction.REVERSE);
        collectorLeft.setDirection(DcMotor.Direction.REVERSE);
        collectorRight.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalExtenderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalExtenderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set encoder mode
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalExtenderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalExtenderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotator.setPosition(0.95);
        grabber.setPosition(0.4);
        pusher.setPosition(0);
        teamMarker.setPosition(0);

        foundationLeft.setPosition(0.4);
        foundationRight.setPosition(0.9);

        mytelemetry.addData("Initialization of Motors", "Complete");
        mytelemetry.update();
        LED.setPosition(0.6795);

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is. **/

        ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

        mytelemetry.addData("Initialization of Vuforia", "Complete");
        mytelemetry.update();
        LED.setPosition(0.6945);

        //Configure Onboard IMU
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();

        parametersIMU.mode                = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        IMULeft.initialize(parametersIMU);
        IMURight.initialize(parametersIMU);
        currentAngle = IMULeft.getAngularOrientation().firstAngle;
        mytelemetry.addData("Gyro Calibration", "Complete");
        mytelemetry.update();
        LED.setPosition(0.7145);

        // Indicates successful initialization
        mytelemetry.addData("Hardware", "Initialized!");
        mytelemetry.addData("Side", Hardware.side);
        mytelemetry.addData("Driver", Hardware.driver);
        mytelemetry.update();
    }
}
