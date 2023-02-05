package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import android.view.View;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.SolomonRandom.MB1242Ex;
import org.firstinspires.ftc.teamcode.SolomonRandom.SensorMBRangeSensor;

import java.util.List;

@Autonomous(name="FibbyAutoNew", group="Robot")
//@Disabled
public class FibbyAutoNew extends LinearOpMode {
    //----------------------------------------------------------------------------------------------------
    // DECLARING MOTOR VARIABLES
    private DcMotor leftFront  = null;
    private DcMotor leftRear   = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear  = null;

    private double leftFrontPower  = 0;
    private double leftRearPower   = 0;
    private double rightFrontPower = 0;
    private double rightRearPower  = 0;

    private DcMotor topLift    = null;  // these are the arm motors
    private DcMotor bottomLift = null;

    private double liftPower = 0;

    private DcMotor parallelEncoder      = null;    // these are the dead wheels
    private DcMotor perpendicularEncoder = null;

    //----------------------------------------------------------------------------------------------------
    // DECLARING SERVO VARIABLES
    private Servo grabber = null;
    private Servo getOutOfMyWay = null;
    
    private Servo deadWheelLift;
    private Servo conePlow;

    public Servo reach = null;

    //----------------------------------------------------------------------------------------------------
    // DECLARING SENSOR VARIABLES
    private IMU imu = null;

    private DistanceSensor distIntake;
    private DistanceSensor distForZero;
    private NormalizedColorSensor frontColorDist;
    View relativeLayout;

    DigitalChannel lowerLimit;
    DigitalChannel upperLimit;

    private MB1242Ex rangeSensor;

    //----------------------------------------------------------------------------------------------------
    // CONSTANT VARIABLES
    static final double P_DRIVE_GAIN = 0.03;    // larger is more responsive, but also less stable

    static final double tickToINCH = 1068;  // reading of the encoder per inch
    
    static final double plowLow  = 0.32;
    static final double plowHigh = 0.24;
    
    static final double openGrabber  = 0.083;
    static final double closeGrabber = 0.03;

    static final int plungeHeight = -140;
    static final int startHeight = 40;

    static final int poleHigh = 1230;
    static final int poleMid = 810;
    static final int poleShort = 470;

    static final int zero = 20;
    static final int tolerance = 10;

    static final int coneStack5 = 220;
    static final int coneStack4 = 165;
    static final int coneStack3 = 130;
    static final int coneStack2 = 120;
    static final int grabHeight = 150;

    static final double reachIn = 0.47;
    static final double reachOut = 0;

    //----------------------------------------------------------------------------------------------------
    // REUSABLE VARIABLES
    int coneImage = 0;  // for camera detection
    
    double corrHeading;
    double heading = 0;
    double diffCorrection;

    boolean questionAnswered = false;
    boolean rightSide = false;

    double desiredCourse = 0;   // telemetry use

    //----------------------------------------------------------------------------------------------------
    // LIGHT CONTROL VARIABLES
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    
    //----------------------------------------------------------------------------------------------------
    // CAMERA/VUFORIA VARIABLES

    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS =
            {
                    "1 Rose",
                    "2 Snail",
                    "3 Pineapple"
            };

    private static final String VUFORIA_KEY = "Ae6MFyH/////AAABmf06rZVcN0VqtDgoLd1KtAcowILYnLgT+SXkGwocAFEpSEmZiGI51vbPAL/QfAgSIhIpPrAK3Fl+vReEgcd1kU5az1pYTI01VqIV1+3ELPbCEcnNMdw3Rs7L8tMMsyfY2nebMlSFfgn6rkfJnnoQEnr4gCGHB1K/7VVZsFg7AlG4SPJ1bORRlhkFf0xIP28Tvr80YnC06hnsL2AQgOJtGrPv7HUO04hWxe9jLMdwHhmnBu/FZfovI9A6EjrzB72joGNTzuLA5R2bBGKW6AsrkKcgw1y50EFOqanZ19gWBX7bc1OExeaNeNIMaGzbMV8jwVahVndqS4EiLc9FuudY21tw4b4jupvhSYUiSGBMtLmh\";\n";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    @Override
    public void runOpMode() {
        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE MOTORS (hardware mapping)
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        parallelEncoder      = hardwareMap.get(DcMotor.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpendicularEncoder");
        
        topLift = hardwareMap.get(DcMotor.class,"Lift");
        bottomLift = hardwareMap.get(DcMotor.class,"Lift2");
        
        // SET THE MOTOR DIRECTION (to make it drive correctly)
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        parallelEncoder.setDirection(DcMotor.Direction.REVERSE);
        perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);

        topLift.setDirection(DcMotor.Direction.REVERSE);
        bottomLift.setDirection(DcMotor.Direction.REVERSE);

        // RESET THE ENCODERS AND SET THE MOTORS TO BRAKE MODE
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ALLOW OR NOT ALLOW ENCODERS
        topLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // SET POWER
        topLift.setPower(0);
        bottomLift.setPower(0);

        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE SERVOS (hardware mapping)
        grabber = hardwareMap.get(Servo .class,"grabber");
        grabber.setPosition(openGrabber);   // set grabber open during init

        conePlow = hardwareMap.get(Servo.class, "plow");
        conePlow.setPosition(plowLow);  // set grabber open during init

        deadWheelLift = hardwareMap.get(Servo.class,"Deadwheel_Lift");
        // deadWheelLift.setPosition(0.5) // it either lift the dead wheels or lowers

        reach = hardwareMap.get(Servo.class, "reach");

        //----------------------------------------------------------------------------------------------------
        // DEFINE AND INITIALIZE SENSORS (hardware mapping)
        imu = hardwareMap.get(IMU.class, "imu");

        distIntake     = hardwareMap.get(DistanceSensor.class, "dist_intake2");
        distForZero    = hardwareMap.get(DistanceSensor.class, "dist_for_zero");
        frontColorDist = hardwareMap.get(NormalizedColorSensor.class, "dist_intake");
        
        lowerLimit = hardwareMap.get(DigitalChannel.class, "l_limit");
        upperLimit = hardwareMap.get(DigitalChannel.class, "u_limit");

        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);

        rangeSensor = hardwareMap.get(MB1242Ex.class, "rangeSensor");

        // DEFINE HUB ORIENTATION
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // INITIALIZE THE IMU WITH THIS MOUNTING ORIENTATION
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // RESET THE HEADING
        resetHeading();

        // LED LIGHTS
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //----------------------------------------------------------------------------------------------------
        // DO DURING INIT
        
        // lower arm until limit switch is activated
        while (lowerLimit.getState() == true){
            topLift.setPower(-0.25);
            bottomLift.setPower(0.25);
        }
        // raise arm unit distance sensor see it
        while (distForZero.getDistance(DistanceUnit.MM) > 80) {
            topLift.setPower(0.4);
            bottomLift.setPower(-0.4);
        }

        //----------------------------------------------------------------------------------------------------
        // QUESTIONS
        while (questionAnswered == false) {
            if (gamepad1.b || gamepad2.b) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                blinkinLedDriver.setPattern(pattern);
                rightSide = true;
                questionAnswered = true;
            } else if (gamepad1.x || gamepad2.x) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                blinkinLedDriver.setPattern(pattern);
                rightSide = false;
                questionAnswered = true;
            } else {
                questionAnswered = false;
            }

            if (questionAnswered == true) {
                telemetry.clear();
                telemetry.update();
            }
        }
        //----------------------------------------------------------------------------------------------------
        // CAMERA/VUFORIA THINGS

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);
        }

        while (!isStarted()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        // this allows un to store the image that camera sees and use it in our cone
                        if (recognition.getLabel() == "1 Rose") {
                            coneImage = 1;
                            if (rightSide == true) {
                                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                                blinkinLedDriver.setPattern(pattern);
                            }
                            else {
                                pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
                                blinkinLedDriver.setPattern(pattern);
                            }
                        }
                        else if (recognition.getLabel() == "2 Snail") {
                            coneImage = 2;
                            pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE;
                            blinkinLedDriver.setPattern(pattern);
                        }
                        else if (recognition.getLabel() == "3 Pineapple") {
                            coneImage = 3;
                            pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
                            blinkinLedDriver.setPattern(pattern);
                        }
                        else {
                            coneImage = 0;
                        }
                    }
                    telemetry.update();
                }
            }
            
            // someone need to explain this to me
            if (gamepad2.dpad_up|| gamepad1.dpad_up) {
                conePlow.setPosition(plowLow);
                liftENC(plungeHeight, -0.5);
                sleep(1000);
            }
            if (topLift.getCurrentPosition() <= -100 && grabber.getPosition() != closeGrabber){
                topLift.setPower(0);
                bottomLift.setPower(0);
                grabber.setPosition(closeGrabber);
                sleep(1000);
                liftENC(startHeight -40, 0.75);
                sleep(750);
                topLift.setPower(0);
                bottomLift.setPower(0);
                conePlow.setPosition(plowHigh);
            }
        }
        
        //----------------------------------------------------------------------------------------------------
        // ENTER CODE BELOW:
        
        sendTelemetry();
    }

    //----------------------------------------------------------------------------------------------------
    // OUR AUTONOMOUS RUNS
    //----------------------------------------------------------------------------------------------------
    

    //----------------------------------------------------------------------------------------------------
    // FUNCTIONS (MYBLOCKS)
    //----------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------
    // GyroStrafeENC
    public void GyroDriveENC(double distance, double power, double course, boolean reset, boolean forward) {
        // for telemetry
        desiredCourse = course;

        // turn off encoders for drive wheels
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // if reset is true, then reset the deal wheel encoders
        if (reset == true) {
            parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // set brake for drive wheels
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // calcuate the motor power for direction
        power = Math.abs(power);           //this ensures that if we are moving forward it is positive
        if (!forward) {power = -power;}    // and if we are going backwards it is negative

        while ((parallelEncoder.getCurrentPosition()/tickToINCH < distance && forward && opModeIsActive()) 
                || // OR
                (parallelEncoder.getCurrentPosition()/tickToINCH > distance && !forward && opModeIsActive())) {
            
            getRawHeading();

            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);

            if (corrHeading < 0) { //robot is drifting to the right, so we need to correct to the left
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power + diffCorrection);
                rightRearPower = (power + diffCorrection);
            }
            else {  //robot is drifting to the left, so we need to correct to the right
                leftFrontPower = (power + diffCorrection);
                leftRearPower = (power + diffCorrection);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power - diffCorrection);
            }

            // ramping down
            if ((Math.abs(distance) - Math.abs(parallelEncoder.getCurrentPosition()/tickToINCH)) <= 3) {
                if (power > 0.2) {

                }
            }

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    //----------------------------------------------------------------------------------------------------
    // GyroStrafeENCStack
    public void GyroDriveENCStack(double distance, double power, double course, boolean reset, boolean forward, int stack) {
        // for telemetry
        desiredCourse = course;

        // turn off encoders for drive wheels
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // if reset is true, then reset the deal wheel encoders
        if (reset == true) {
            parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // set brake for drive wheels
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set conePlow to low so it will not break
        conePlow.setPosition(plowLow);

        while ((stack == 1 &&  (distIntake.getDistance(DistanceUnit.MM) <= 50 || distIntake.getDistance(DistanceUnit.MM) >= 67))  // single cone
                ||  // OR
                (stack >= 2 && (distIntake.getDistance(DistanceUnit.MM) <= 30 || distIntake.getDistance(DistanceUnit.MM) >= 50))// stack of Cones
                && 
                (opModeIsActive())) {
            
            // calcuate the motor power for direction
            power = Math.abs(power);           //this ensures that if we are moving forward it is positive
            if (!forward) {power = -power;}    // and if we are going backwards it is negative
    
            while ((parallelEncoder.getCurrentPosition()/tickToINCH < distance && forward && opModeIsActive())
                    ||
                    (parallelEncoder.getCurrentPosition()/tickToINCH > distance && !forward && opModeIsActive())) {
                getRawHeading();
    
                corrHeading = course - heading;
                diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);
    
                if (corrHeading < 0) { //robot is drifting to the right, so we need to correct to the left
                    leftFrontPower = (power - diffCorrection);
                    leftRearPower = (power - diffCorrection);
                    rightFrontPower = (power + diffCorrection);
                    rightRearPower = (power + diffCorrection);
                } else {  //robot is drifting to the left, so we need to correct to the right
                    leftFrontPower = (power + diffCorrection);
                    leftRearPower = (power + diffCorrection);
                    rightFrontPower = (power - diffCorrection);
                    rightRearPower = (power - diffCorrection);
                }
    
                // ramping down
                if ((Math.abs(distance) - Math.abs(parallelEncoder.getCurrentPosition() / tickToINCH)) <= 3) {
                    if (power > 0.2) {
    
                    }
                }
    
                leftFront.setPower(leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    
    
    //----------------------------------------------------------------------------------------------------
    // GyroStrafeENC
    public void GyroStrafeENC(double distance, double power, String direction, double course) {
        // for telemetry
        desiredCourse = course;

        distance = Math.abs(distance);
        //direction = toString().toLowerCase(direction);
        
        // turn off encoders for drive wheels
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reset the deal wheel encoders
        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set brake for drive wheels
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(perpendicularEncoder.getCurrentPosition()/tickToINCH) <= distance && (opModeIsActive())) {
            getRawHeading();

            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);

            if ((corrHeading < 0 && direction == "left") || (corrHeading > 0 && direction == "right")) {
                leftFrontPower = (power + diffCorrection);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power + diffCorrection);
                rightRearPower = (power - diffCorrection);
            }
            else {
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power + diffCorrection);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power + diffCorrection);
            }

            if(direction == "left") {
                leftFront.setPower(-leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(-rightRearPower);
            }
            else if(direction == "right") {
                leftFront.setPower(leftFrontPower);
                leftRear.setPower(-leftRearPower);
                rightFront.setPower(-rightFrontPower);
                rightRear.setPower(rightRearPower);
            }
            else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    
    //----------------------------------------------------------------------------------------------------
    // GyroSpin
    public void GyroSpin(double power, double course) {
        // for telemetry
        desiredCourse = course;

        // turn off encoders for drive wheels
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // set break to drive wheels
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        getRawHeading();

        corrHeading = course - heading;
        diffCorrection = Math.abs(corrHeading * P_DRIVE_GAIN);
        
        if (heading >= course) {
            while (heading >= course) {
                leftFrontPower = (-power);
                leftRearPower = (-power);
                rightFrontPower = (power);
                rightRearPower = (power);

                leftFront.setPower(leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);
                getRawHeading();
            }
        }
        else if (heading <= course) {
            while (heading <= course) {
                leftFrontPower = (power);
                leftRearPower = (power);
                rightFrontPower = (-power);
                rightRearPower = (-power);

                leftFront.setPower(leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(rightRearPower);
                getRawHeading();
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    
    //----------------------------------------------------------------------------------------------------
    // liftENC
    public void liftENC(int distance, double power) {
        if (distance > 1300) {distance = 1300;}
        
        topLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLift.setTargetPosition(distance);
        bottomLift.setTargetPosition(-distance);
        
        topLift.setPower(power);
        bottomLift.setPower(-power);
        
        topLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* ALL IN THIS COMMENT IS OLD CODE FOR OPEN DRIVE
        if ((power > 0 && lift.getCurrentPosition() <= distance) || 
                (power < 0 && lift.getCurrentPosition() >= distance + 15 && lowerLimit.getState() == true) && 
                        (opModeIsActive())) {
            lift.setPower(power);
            lift2.setPower(-power);
        } else {
            lift.setPower(0);
            lift2.setPower(0);
        }
            
        if ((power < 0 && lift.getCurrentPosition() <= distance) || 
                (power > 0 && lift.getCurrentPosition() >= distance + 15 && lowerLimit.getState() == true) && 
                        (opModeIsActive())) {
                lift.setPower(0);
                lift2.setPower(0);
        }
        */
    }

    //----------------------------------------------------------------------------------------------------
    // CAMERA/VUFORIA FUNCTIONS
    private void initVuforia()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    //----------------------------------------------------------------------------------------------------
    // GYRO FUNCTIONS
    public void getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        heading = -orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    private void sendTelemetry() {
        telemetry.addData("Luxo", "Telemetry");
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", desiredCourse, heading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", corrHeading, diffCorrection);

        telemetry.addLine();

        telemetry.addData("Actual Pos Front L:R",  "%7d:%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
        telemetry.addData("Actual Pos Rear  L:R",  "%7d:%7d", leftRear.getCurrentPosition(),  rightRear.getCurrentPosition());

        telemetry.addLine();

        telemetry.addData("Wheel Speeds Front L:R.", "%5.2f : %5.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Wheel Speeds Rear  L:R.", "%5.2f : %5.2f", leftRearPower,  rightRearPower);

        telemetry.addLine();

        telemetry.addData("rangeSensor", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

    }
}
