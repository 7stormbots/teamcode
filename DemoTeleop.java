package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Demo Teleop", group = "")


public class DemoTeleop extends LinearOpMode {

    private CRServo spinWheels;
    private Servo  tiltUp, swingWide;
    private ColorSensor sensorColorRange_REV_ColorRangeSensor;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private DcMotor Intake;
    private TouchSensor palm;
    private boolean GamepadBisPressed;
    private boolean Break;
    private Servo Claw;
    private boolean armGoLeft = false;
    public static final double NEW_P = 20;
    public static final double NEW_I = 15;
    private Servo Blinkers;
    public static final double NEW_D = 0;
    public static final double NEW_F = 5;
    private DcMotor Arm;
    private boolean GamepadUpisPressed;
    private boolean GamepadDownisPressed;
    private CRServo leftSweeper;
    private CRServo rightSweeper;
    private boolean wegood;
    private boolean GamepadLeftisPressed;
    private boolean gamepad2leftdpadisPressed;
    private boolean gamepad2rightdpadisPressed;
    public double grabberClose = 0.6;
    public double grabberOpen = 0.4;
    public double grabberIntakeOpen = 0.375;
    private int armTopBehind = -2420;
    private int armMiddleBehind = -2702;
    private int armBottomBehind = -3000;
    public double grabberSuperOpen = 0.2;
    public boolean armFailSafe = false;

    public boolean gamepad1aispressed = false;

    public boolean slowmode = true;
    private boolean GamepadRightisPressed;
    private boolean sharedHub = false;
    private boolean Gamepad1dpaddown = false;
    private boolean GamepadYisPressed;
    ElapsedTime TimerE;
    private boolean GamepadAisPressed;
    private RevBlinkinLedDriver blinkin;
    private boolean Clawhasbeenopened;
    //private DcMotor Spinner;
    private int tickPerDegree = 10;
    private double tickPerExtraLength = 1950 / 15;
    private double height = 11;
    private double distanceFromBot = 15;
    private double Gamepad2Current;
    private double Gamepad2Previous;

    private boolean gamepadleftbumper;
    private int armBottomFront = -240;
    private int armMiddleFront = -530;
    private int armTopFront = -890;
    private boolean GamepadxisPressed;
    private State CurrentState;
    boolean GamepadbisPressed = false;
    boolean Palm = false;
    boolean Test = false;
    boolean ArmGoingup = false;
    boolean Intakeon = false;
    double Position = 2;
    boolean gamepadBumperLeftisPressed;
    boolean gamepadBumperRightisPressed;
    private double WristDown = 0.65;
    private double ServoPosition = 0;
    private double WristUp = 0;
    private int Increment = 75;
    private double NoCAP = 1;
    ElapsedTime TimerA;
    private int extensionPosition;
    ElapsedTime TimerB;
    private DcMotor Spinner;
    ElapsedTime TimerC;
    ElapsedTime TimerD;
    ElapsedTime TimerF;
    ElapsedTime TimerG;
    ElapsedTime TimerAndrewsHot;
    ElapsedTime TimerBalls;
    ElapsedTime extensionTimer;
    ElapsedTime extensionChecker;
    private int currentArmPosition;
    private DigitalChannel RedLED2;
    private DigitalChannel GreenLED2;
    private DigitalChannel RedLED;
    private DigitalChannel GreenLED;
    private DcMotorEx extension;
    private boolean extensionAtHome = false;
    private int Retracted = 0;
    private int Home = 0;
    private int partialExtension = -800;
    //  private int halfExtension =
    // private int Maximum = 442;
    private int Maximum = -1900;
    //private int Maximum = -1950;
    private int TopLevelLeft = -895;
    private int TopLevelRight = 895;
    private boolean armAtHome = false;
    private boolean Rezero = false;
    private double FilteredXR, LastFilteredXR, FilteredXL, LastFilteredXL, FilteredYL, LastFilteredYL;
    boolean gamepad2xIsPressed;
    boolean gamepad2yIsPressed;
    boolean isGamepad1dpaddownIsPressed;
    private Servo lefteye;
    private Servo righteye;

    private enum State {
        DEFAULT,
        INTAKE,
        DROP,
    }

    private class IntakeStateMachine implements Runnable {
        public IntakeStateMachine() {
        }

        public void run() {
            while (opModeIsActive()) {
                ultimateState();
            }
        }
    }


    @Override
    public void runOpMode() {

        TimerA = new ElapsedTime();

        TimerB = new ElapsedTime();

        TimerC = new ElapsedTime();

        TimerD = new ElapsedTime();

        TimerE = new ElapsedTime();

        TimerF = new ElapsedTime();

        TimerG = new ElapsedTime();

        TimerAndrewsHot = new ElapsedTime();

        TimerBalls = new ElapsedTime();

        extensionTimer = new ElapsedTime();

        extensionChecker = new ElapsedTime();

        /*RedLED = hardwareMap.get(DigitalChannel.class, "Red");
        GreenLED = hardwareMap.get(DigitalChannel.class, "Green");*/
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftSweeper = hardwareMap.get(CRServo.class, "leftSweeper");
        rightSweeper = hardwareMap.get(CRServo.class, "rightSweeper");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Blinkers = hardwareMap.get(Servo.class, "Blinkers");
        sensorColorRange_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        //Spinner = hardwareMap.get(DcMotor.class, "Spinner");
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        swingWide = hardwareMap.get(Servo.class,"swingWide");
        tiltUp = hardwareMap.get(Servo.class, "tiltUp" );
        spinWheels = hardwareMap.get(CRServo.class, "spinWheels");


        //Spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CurrentState = State.DEFAULT;
        IntakeStateMachine myThread = new IntakeStateMachine();
        myThread.run();

        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        leftSweeper.setDirection(CRServo.Direction.REVERSE);

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        palm = hardwareMap.get(TouchSensor.class, "palm");
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        //  extension.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);


        // **** Arm homing is currently inactive.  This needs to be enabled to ensure that our arm setpoints are correct.  Probably want to do this before setting the arm mode (after the arm hardwaremap statement)
        /*
        currentArmPosition = 100;
        while (currentArmPosition - Arm.getCurrentPosition() > 0 || currentArmPosition - Arm.getCurrentPosition() < 0) {
            Arm.setPower(-0.2);
            sleep(50);
            currentArmPosition = Arm.getCurrentPosition();
        }

 */
        Spinner = hardwareMap.get(DcMotor.class, "Spinner");
        Spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenLED = hardwareMap.get(DigitalChannel.class, "green");
        RedLED = hardwareMap.get(DigitalChannel.class, "red");
        GreenLED2 = hardwareMap.get(DigitalChannel.class, "green2");
        RedLED2 = hardwareMap.get(DigitalChannel.class, "red2");
        RedLED.setMode(DigitalChannel.Mode.OUTPUT);
        GreenLED.setMode(DigitalChannel.Mode.OUTPUT);
        RedLED2.setMode(DigitalChannel.Mode.OUTPUT);
        GreenLED2.setMode(DigitalChannel.Mode.OUTPUT);
        GreenLED.setState(false);
        GreenLED2.setState(false);
        RedLED.setState(false);
        RedLED2.setState(false);
        lefteye = hardwareMap.get(Servo.class, "leftEye");
        righteye = hardwareMap.get(Servo.class, "rightEye");
        // Homing();
        LastFilteredXL = 0;
        LastFilteredXR = 0;
        LastFilteredYL = 0;
        lefteye.setPosition(0.5);
        righteye.setPosition(0.5);
        waitForStart();
        extensionTimer.reset();
        extension.setPower(0.5);
        sleep(2000);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(0);
        extension.setPower(1);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            //leftSweeper.setPower(Intake.getPower());
            //rightSweeper.setPower(-Intake.getPower());
            moveEyes();
            //MecanumFunction(FilteredYL, FilteredXL, FilteredXR);


            if (gamepad2.x && !gamepad2xIsPressed){
                FilteredYL = 0.1 * gamepad2.left_stick_y + 0.9 * LastFilteredYL;
                //FilteredXL = 0.1 * gamepad2.left_stick_x + 0.9 * LastFilteredXL;
                FilteredXR = 0.1 * gamepad2.right_stick_x + 0.9 * LastFilteredXR;


                leftFront.setPower(1 * (FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR)));
                rightFront.setPower(1 * (-(FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR))));
                leftRear.setPower(1 * ((FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));
                rightRear.setPower(1 * (-(FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));

                LastFilteredYL = FilteredYL;
                LastFilteredXL = FilteredXL;
                LastFilteredXR = FilteredXR;
            } else if (gamepad2.y && !gamepad2yIsPressed){
                FilteredYL = 0.1 * gamepad2.left_stick_y + 0.9 * LastFilteredYL;
                //FilteredXL = 0.1 * gamepad2.left_stick_x + 0.9 * LastFilteredXL;
                FilteredXR = 0.1 * gamepad2.right_stick_x + 0.9 * LastFilteredXR;

                leftFront.setPower(-1 * (FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR)));
                rightFront.setPower(-1 * (-(FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR))));
                leftRear.setPower(-1 * ((FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));
                rightRear.setPower(-1 * (-(FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));

                LastFilteredYL = FilteredYL;
                LastFilteredXL = FilteredXL;
                LastFilteredXR = FilteredXR;
            }
            if (sharedHub){
                telemetry.addData("extensionatHome?", extensionAtHome);
                telemetry.update();
            } else {
                telemetry.addData("extensionatHome?", extensionAtHome);
                telemetry.update();
            }
            telemetry.update();
            gamepad2xIsPressed = gamepad2.x;
            gamepad2yIsPressed = gamepad2.y;

            if (gamepad2.a && !GamepadUpisPressed){
                if (slowmode) {
                    slowmode = false;
                } else {
                    slowmode = true;
                }
            }
            if (!armGoLeft){
                GreenLED2.setState(false);
                RedLED2.setState(false);
                if (TimerF.milliseconds() < 200) {
                    GreenLED.setState(false);
                    RedLED.setState(false);
                }
                else if (TimerF.milliseconds() < 400) {
                    GreenLED.setState(true);
                    RedLED.setState(true);
                } else {
                    TimerF.reset();
                }
            }
            else{
                RedLED.setState(false);
                GreenLED.setState(false);
                if (TimerF.milliseconds() < 200) {
                    GreenLED2.setState(false);
                    RedLED2.setState(false);
                }
                else if (TimerF.milliseconds() < 400) {
                    GreenLED2.setState(true);
                    RedLED2.setState(true);
                } else {
                    TimerF.reset();
                }
            }

            GamepadUpisPressed = gamepad2.a;
            ////////////////////////////////////////////////////
            // Make a MECANUM function to hide all this ugliness - Coach David
            ////////////////////////////////////////////////////
            if (armGoLeft) {
                Blinkers.setPosition(0.7);
            } else {
                Blinkers.setPosition(0.25);
            }



            ultimateState();
            telemetry.addData("extensionatHome?", extensionAtHome);
            telemetry.update();
            if (slowmode) {
                /*
                leftFront.setPower((0.46 * ((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)) - (0.35 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                rightFront.setPower((0.46 * (-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)) - (0.35 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                leftRear.setPower((0.46* ((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)) - (0.35 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                rightRear.setPower((0.46 * (-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)) - (0.35 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));


                 */
                FilteredYL = 0.3 * gamepad2.left_stick_y + 0.7 * LastFilteredYL;
                //FilteredXL = 0.3 * gamepad2.left_stick_x + 0.7 * LastFilteredXL;
                FilteredXR = 0.3 * gamepad2.right_stick_x + 0.7 * LastFilteredXR;


                leftFront.setPower(0.6 * ((FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR))));
                rightFront.setPower(0.6 * ((-(FilteredYL) - (FilteredXL) - (0.75 * (FilteredXR)))));
                leftRear.setPower(0.6 * (((FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR)))));
                rightRear.setPower(0.6 * (-(FilteredYL) + (FilteredXL) - (0.75 * (FilteredXR))));

                LastFilteredYL = FilteredYL;
                LastFilteredXL = FilteredXL;
                LastFilteredXR = FilteredXR;

            } else {
                leftFront.setPower(1 * ((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (0.75 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                rightFront.setPower(1 * (-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (0.75 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                leftRear.setPower(1 * ((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (0.75 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                rightRear.setPower(1 * (-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (0.75 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));

            }

            if (palm.isPressed()) {
                telemetry.addData("I'm pressed aka mad", 1);
            } else {
                telemetry.addData("I'm NOT pressed aka not mad", 1);
            }
            telemetry.addData("CurrentState: balls", CurrentState);
            telemetry.addData("Dist to tgt (cm)", ((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));

            telemetry.update();
/*
            if (gamepad1.b){
                Claw.setPosition(grabberOpen);

                //greenLED.setState(false);
                //redLED.setState(true);
            } else {
                Claw.setPosition(grabberClose);

                //greenLED.setState(true);
                //redLED.setState(false);
            }

 */
/*
            GamepadAisPressed = gamepad1.a;  // Setting this status here is not best practice and it might cause issues from time to time.  This statement is correct but should be somewhere else
            GamepadYisPressed = gamepad1.y;  // Setting this status here is not best practice and it might cause issues from time to time.  This statement is correct but should be somewhere else
            if (gamepad1.dpad_up && !GamepadUpisPressed) {
                Arm.setTargetPosition(armMiddleBehind);
                Arm.setPower(-.8);

            } else if (gamepad1.dpad_left && !GamepadLeftisPressed) {
                Arm.setTargetPosition(armBottomFront);
                Arm.setPower(-.8);
            }
            else if (gamepad1.dpad_right && !GamepadRightisPressed) {
                Arm.setTargetPosition(armTopBehind);
                Arm.setPower(-.8);
            }
            else if (gamepad1.dpad_down && !GamepadDownisPressed) {
                Arm.setTargetPosition(5);
                Arm.setPower(0.75);
            }
            GamepadUpisPressed = gamepad1.dpad_up;      // Setting this status here is not best practice and it might cause issues from time to time.  This statement is correct but should be somewhere else
            GamepadDownisPressed = gamepad1.dpad_down;  // Setting this status here is not best practice and it might cause issues from time to time.  This statement is correct but should be somewhere else
            GamepadRightisPressed = gamepad1.dpad_right;// Setting this status here is not best practice and it might cause issues from time to time.  This statement is correct but should be somewhere else
            GamepadLeftisPressed = gamepad1.dpad_left;  // Setting this status here is not best practice and it might cause issues from time to time.  This statement is correct but should be somewhere else

            if (gamepad1.x && !GamepadxisPressed){
                Arm.setTargetPosition(armTopBehind);
                Arm.setPower(-0.5);
            }
            gamepadleftbumper = gamepad1.left_bumper; // This is not in the right spot.  This will almost always never work as it is written right now.  Why is that?
            if (gamepad1.left_bumper && !gamepadleftbumper){
                Arm.setTargetPosition(-1111);
                Arm.setPower(-0.75);
            }
            if (Arm.getCurrentPosition() > -10){
           //     Intake.setPower(-1);
                Wrist.setPosition(1);
             //   Claw.setPosition(grabberHalfOpen);
            } else {
                Wrist.setPosition(0);
             //   Intake.setPower(1);
               // Claw.setPosition(grabberClose);
            }
            if (!Spinner.isBusy()) {

            }
            GamepadxisPressed = gamepad1.x; // Setting this status here is not best practice and it might cause issues from time to time.  This statement is correct but should be somewhere else


*/

            //Spinner.setPower(gamepad1.left_trigger * .7);
        }
    }

    private void Homing() {
        Claw.setPosition(grabberClose);
        currentArmPosition = 100;
        // Wrist.setPosition(WristDown);
        Intake.setPower(1);
        leftFront.setPower((0.5));
        rightFront.setPower((0.5));
        leftRear.setPower((0.5));
        rightRear.setPower((0.5));
        sleep(1500);
        leftFront.setPower((0));
        rightFront.setPower((0));
        leftRear.setPower((0));
        rightRear.setPower((0));
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TimerC.reset();
        while (TimerC.milliseconds() < 3000) {
            Arm.setPower(0.4);
            currentArmPosition = Arm.getCurrentPosition();
            telemetry.addData("Andy NUTZ awoogaa", TimerC.milliseconds());
            telemetry.update();
        }
        Intake.setPower(0);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void MecanumFunction(double YL, double XL, double XR){
        leftFront.setPower((YL - XL + XR));
        rightFront.setPower((YL + XL - XR));
        leftRear.setPower((YL + XL + XR));
        rightRear.setPower((YL - XL - XR));

        FilteredYL = 0.1 * gamepad2.left_stick_y + 0.9 * LastFilteredYL;
        //FilteredXL = 0.1 * gamepad2.left_stick_x + 0.9 * LastFilteredXL;
        FilteredXR = 0.1 * gamepad2.right_stick_x + 0.9 * LastFilteredXR;

        LastFilteredYL = FilteredYL;
        LastFilteredXL = FilteredXL;
        LastFilteredXR = FilteredXR;
    }
    private void moveEyes() {
        /*
        ElapsedTime Eyes;
        Eyes = new ElapsedTime();
        double inc = 0;
        if (Eyes.milliseconds() < 500) {
            while (lefteye.getPosition() > inc) {
                lefteye.setPosition(lefteye.getPosition() - 0.1);
                righteye.setPosition(righteye.getPosition() - 0.1);
            }
        } else if (Eyes.milliseconds() < 1000){
            inc = 1;
            while (lefteye.getPosition() < inc) {
                lefteye.setPosition(lefteye.getPosition() + 0.1);
                righteye.setPosition(righteye.getPosition() + 0.1);
            }
        } else{
            inc = 0;
            Eyes.reset();

         */
        if (gamepad2.right_stick_x < 0.1) {
            lefteye.setPosition(0);
            righteye.setPosition(0);
        } else if (gamepad2.right_stick_x > -0.1){
            lefteye.setPosition(1);
            righteye.setPosition(1);
        } else {
            lefteye.setPosition(0.5);
            righteye.setPosition(0.5);
        }
    }

    //currentArmPosition - Arm.getCurrentPosition() > 0 || currentArmPosition - Arm.getCurrentPosition() < 0
    private void ultimateState() {

       /*
        boolean GamepadYisPressed = false;
        boolean GamepadLeftisPressed = false;
        boolean GamepadRightisPressed = false;
        boolean GamepadbisPressed = false;
        boolean Palm = false;
        boolean Test = false;
        boolean Intakeon = false;
        double Position = 2;
        */


        gamepadBumperLeftisPressed = gamepad1.left_bumper;
        gamepadBumperRightisPressed = gamepad1.right_bumper;

        GamepadxisPressed = gamepad1.x;

        switch (CurrentState) {
            case DEFAULT:
                if (Intake.getPower() >= -.1) {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                   Spinner.setPower(0.75 * (gamepad1.right_trigger - gamepad1.left_trigger));
                    if (!armFailSafe) {
                        if (gamepad1.a && !gamepad1aispressed){
                            armFailSafe = true;
                        }
                        gamepad1aispressed = gamepad1.a;
                        if (gamepad2.right_bumper && !gamepad2leftdpadisPressed) {
                            armGoLeft = false;
                        } else if (gamepad2.left_bumper && !gamepad2rightdpadisPressed) {
                            armGoLeft = true;
                        }
                        gamepad2leftdpadisPressed = gamepad2.left_bumper;
                        gamepad2rightdpadisPressed = gamepad2.right_bumper;
                        /*
                        if (!extensionAtHome) {
                            telemetry.addData("tryna reset!", true);
                            telemetry.addData("extensionposition", extensionPosition);
                            telemetry.update();
                            extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            extension.setPower(1);
                            if (!((extension.getCurrentPosition() + extensionPosition) > -3)) {
                                extensionChecker.reset();
                            }
                            if (extensionChecker.milliseconds() > 250){
                                extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                extension.setTargetPosition(0);
                                extension.setPower(1);
                                extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                extensionAtHome = true;

                            }
                            if (extensionTimer.milliseconds() > 200){
                                extensionPosition = extension.getCurrentPosition();
                                extensionTimer.reset();
                            }
                            }


                         */
                        extension.setTargetPosition(Retracted);
                        extension.setPower(1);
                        if (Arm.getCurrentPosition() < 50 || Arm.getCurrentPosition() > -50) {
                            armAtHome = true;
                        }
                        if (extension.getCurrentPosition() > -50) {
                            Arm.setTargetPosition(Home);
                            if ((Math.abs(Arm.getCurrentPosition()) < 400)) {
                                Arm.setPower(0.5);
                            } else if ((Math.abs(Arm.getCurrentPosition()) < 20)) {
                                Arm.setPower(0.75);
                            } else {
                                Arm.setPower(0.75);

                            }
                        }


                        Claw.setPosition(grabberOpen);
                        if (extension.getCurrentPosition() > -20) {
                            if (gamepad1.y) {
                                Intake.setPower(-1);
                                leftSweeper.setPower(1);
                                rightSweeper.setPower(1);
                                Rezero = false;
                            } else {
                                Intake.setPower(1);
                                leftSweeper.setPower(-1);
                                rightSweeper.setPower(-1);
                            }
                        }
                    } else {
                        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        if (gamepad1.dpad_up){
                            extension.setTargetPosition(0);
                            extension.setPower(1);
                        }
                        if (gamepad1.dpad_down){
                            extension.setTargetPosition(partialExtension);
                            extension.setPower(1);
                        }
                        if (gamepad1.dpad_left) {
                            Arm.setPower(-0.269);
                        }
                        else if (gamepad1.dpad_right){
                            Arm.setPower(0.269);
                        } else {
                            Arm.setPower(0);
                        }
                        if (gamepad1.b){
                            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            Arm.setTargetPosition(0);
                            Arm.setPower(1);
                            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            extension.setTargetPosition(0);
                            extension.setPower(1);
                            armFailSafe = false;
                        }
                        if (gamepad1.a && !gamepad1aispressed){
                            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            Arm.setTargetPosition(0);
                            Arm.setPower(1);
                            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            armFailSafe = false;
                        }
                        gamepad1aispressed = gamepad1.a;
                    }


                } else if (!Palm && (Intake.getPower() != 0) && (armAtHome)) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.DROP;
                }
                break;

            case INTAKE:
                if (!Palm && (Intake.getPower() != 0) && (armAtHome)) {
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    extensionAtHome = false;
                    /*
                    if (gamepad1.dpad_left && TimerAndrewsHot.milliseconds() > 50){
                        swingWide.setPosition(swingWide.getPosition() + 0.02);
                        TimerAndrewsHot.reset();
                    }
                    if (gamepad1.dpad_right && TimerAndrewsHot.milliseconds() > 50){
                        swingWide.setPosition((swingWide.getPosition() - 0.02));
                        TimerAndrewsHot.reset();
                    }
                    if (gamepad1.dpad_up && TimerBalls.milliseconds() > 50){
                        tiltUp.setPosition(tiltUp.getPosition() + 0.01);
                        TimerBalls.reset();
                    }
                    if (gamepad1.dpad_down && TimerBalls.milliseconds() > 50) {
                        tiltUp.setPosition(tiltUp.getPosition() - 0.0025);
                        TimerBalls.reset();
                    }
                    if (gamepad1.x){
                        spinWheels.setPower(1);
                    } else if (gamepad1.y){
                        spinWheels.setPower(-1);
                    } else {
                        spinWheels.setPower(0);
                    }

                     */
                    if (gamepad1.dpad_down) {
                        Intake.setPower(0);
                        leftSweeper.setPower(0);
                        rightSweeper.setPower(0);
                    } else {
                        if (extension.getCurrentPosition() < -160){
                            Intake.setPower(-1);
                            leftSweeper.setPower(1);
                            rightSweeper.setPower(1);
                            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            Arm.setTargetPosition(0);
                            Arm.setPower(1);
                            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }

                    }
                    if (gamepad2.right_bumper && !gamepad2leftdpadisPressed) {
                        armGoLeft = false;
                    } else if (gamepad2.left_bumper && !gamepad2rightdpadisPressed) {
                        armGoLeft = true;
                    }
                    gamepad2leftdpadisPressed = gamepad2.left_bumper;
                    gamepad2rightdpadisPressed = gamepad2.right_bumper;
                    telemetry.addData("distance", ((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
                    telemetry.addData("timer", TimerB);
                    telemetry.update();

                    if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > 6)) {
                        TimerB.reset();

                        telemetry.addData("Palm?!?!?!?!?!??!?!?!??!", "None");
                    }


                    Spinner.setPower(0.75 * (gamepad1.right_trigger - gamepad1.left_trigger));

                    if (TimerB.milliseconds() >= 500) {
                        Palm = true;
                        Intake.setPower(0);
                        leftSweeper.setPower(0);
                        rightSweeper.setPower(0);
                        armAtHome = false;
                    }

                    if (TimerB.milliseconds() >= 250) {
                        Claw.setPosition(grabberClose);
                    } else {
                        Claw.setPosition(grabberIntakeOpen);
                    }

                    if ((Arm.getCurrentPosition() - Home) < 10 || (Arm.getCurrentPosition() + Home) > -10) {
                        extension.setTargetPosition(partialExtension);
                        extension.setPower(0.75);
                    }
                    if ((partialExtension - extension.getCurrentPosition() >= -20)) {

                        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Arm.setTargetPosition(0);
                        Arm.setPower(1);
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    Arm.setTargetPosition(Home);
                    Arm.setPower(1);

                } else if (Palm) {
                    CurrentState = State.DROP;
                } else {
                    CurrentState = State.DEFAULT;
                }

                break;

            case DROP:
                if (Palm) {
                    extensionAtHome = false;
                    if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 6)) {
                        TimerB.reset();

                        telemetry.addData("Palm?", "None");
                    }
                    if (TimerB.milliseconds() >= 500) {
                        Palm = false;

                    }
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    if (!sharedHub) {

                        if (gamepad1.x) {
                            sharedHub = true;
                        } else if (gamepad1.b) {
                            sharedHub = false;
                        }
                        armAtHome = false;
                        if (gamepad1.a) {
                            Claw.setPosition(grabberSuperOpen);
                        } else {
                            Claw.setPosition(grabberClose);
                        }

                        if (!Palm || gamepad2.dpad_down) {
                            Palm = false;
                            Intake.setPower(0.1);
                            leftSweeper.setPower(0);
                            rightSweeper.setPower(0);
                        } else {
                            if (Math.abs(Arm.getCurrentPosition()) < 50){
                                Intake.setPower(-0.1);
                                leftSweeper.setPower(0.1);
                                rightSweeper.setPower(0.1);
                            } else {
                                Intake.setPower(1);
                                leftSweeper.setPower(-1);
                                rightSweeper.setPower(-1);
                            }

                        }
                        if (Arm.getCurrentPosition() < -800 || Arm.getCurrentPosition() > 800) {
                            if (!armGoLeft && Arm.getCurrentPosition() > -800) {
                                extension.setTargetPosition(partialExtension);
                                extension.setPower(1);
                            } else if (armGoLeft && Arm.getCurrentPosition() < 800) {
                                extension.setTargetPosition(partialExtension);
                                extension.setPower(1);
                            } else {
                                extension.setTargetPosition(0);
                                extension.setPower(1);
                            }
                        } else {
                            extension.setTargetPosition(0);
                            extension.setPower(1);

                        }
                        if (gamepad2.right_bumper && !gamepad2leftdpadisPressed) {
                            armGoLeft = false;
                        } else if (gamepad2.left_bumper && !gamepad2rightdpadisPressed) {
                            armGoLeft = true;
                        }
                        gamepad2leftdpadisPressed = gamepad2.left_bumper;
                        gamepad2rightdpadisPressed = gamepad2.right_bumper;
                        if (armGoLeft) {
                            telemetry.addData("Currently in regular drop left mode", 1);
                            telemetry.update();
                            if (extension.getCurrentPosition() >= -20) {
                                Arm.setTargetPosition(TopLevelLeft);
                                Arm.setPower(1);

                            }
                        } else {
                            telemetry.addData("Currently in regular drop right mode", 1);
                            telemetry.update();

                            if (extension.getCurrentPosition() >= -20) {
                                Arm.setTargetPosition(TopLevelRight);
                                Arm.setPower(1);


                            }
                        }

                    } /*else if (sharedHub) {
                        /*
                       if (extension.getCurrentPosition() > -50){
                            Intake.setPower(1);
                            leftSweeper.setPower(-1);
                            rightSweeper.setPower(-1);
                       }


                        if (Math.abs(Arm.getCurrentPosition()) > 300){
                            Intake.setPower(1);
                            leftSweeper.setPower(-1);
                            rightSweeper.setPower(-1);
                        } else {
                            Intake.setPower(-1);
                            leftSweeper.setPower(-1);
                            rightSweeper.setPower(-1);
                        }
                       armAtHome = false;
                       if (gamepad1.x) {
                           sharedHub = true;
                       } else if (gamepad1.b) {
                           sharedHub = false;
                       }
                       telemetry.addData("Currently in shared hub left mode", 1);
                       telemetry.update();
                       if (gamepad1.a) {
                           Claw.setPosition(grabberSuperOpen);
                       } else {
                           Claw.setPosition(grabberClose);
                       }
                       if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 4)) {
                           TimerB.reset();

                           telemetry.addData("Palm??!?!?!?!?!?!?!?!?!?!?!?!?", "None");
                       }
                       if (TimerB.milliseconds() >= 250 || gamepad1.dpad_down) {
                           Palm = false;
                           Intake.setPower(0);
                           leftSweeper.setPower(0);
                           rightSweeper.setPower(0);
                       } else {

                        }

                       if (gamepad2.right_bumper && !gamepad2leftdpadisPressed) {
                           armGoLeft = false;
                       } else if (gamepad2.left_bumper && !gamepad2rightdpadisPressed) {
                           armGoLeft = true;
                       }
                       gamepad2leftdpadisPressed = gamepad2.left_bumper;
                       gamepad2rightdpadisPressed = gamepad2.right_bumper;
                       if (armGoLeft) {
                           if (gamepad1.dpad_left && distanceFromBot > 13) { //&& (Math.abs(((int)((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Arm.getCurrentPosition()) < 10) && Math.abs((int)((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength)) - extension.getCurrentPosition() < 10) {
                               if (TimerE.milliseconds() > 50) {
                                   distanceFromBot -= .5;
                                   TimerE.reset();
                               }
                           } else if (gamepad1.dpad_right && distanceFromBot < 26) { //&& (Math.abs(((int)((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Arm.getCurrentPosition()) < 10) && Math.abs((int)((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength)) - extension.getCurrentPosition() < 10) {
                               if (TimerE.milliseconds() > 50) {
                                   distanceFromBot += .5;
                                   TimerE.reset();
                               }
                           }
                           if (extension.getCurrentPosition() > -75 || extension.getCurrentPosition() < -700) {
                               Arm.setTargetPosition((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * -tickPerDegree);
                               Arm.setPower(0.4);
                           }

                           if (armGoLeft && Arm.getCurrentPosition() < 0) {
                               if (Math.abs(((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Math.abs(Arm.getCurrentPosition())) < 100) {
                                   extension.setTargetPosition((int) ((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * -tickPerExtraLength));
                                   extension.setPower(0.75);
                               } else {
                                   extension.setTargetPosition(0);
                                   extension.setPower(1);
                               }
                           } else if (!armGoLeft && Arm.getCurrentPosition() > 0) {
                               if (Math.abs(((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Math.abs(Arm.getCurrentPosition())) < 100) {
                                   extension.setTargetPosition((int) ((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * -tickPerExtraLength));
                                   extension.setPower(0.75);
                               } else {
                                   extension.setTargetPosition(0);
                                   extension.setPower(1);
                               }
                           } else {
                               extension.setTargetPosition(0);
                               extension.setPower(1);
                           }
                       } else {
                           telemetry.addData("Currently in shared hub right mode", 1);
                           armAtHome = false;
                           /*
                           if (extension.getCurrentPosition() > -400){
                               Intake.setPower(-0.1);
                               leftSweeper.setPower(0.1);
                               rightSweeper.setPower(0.1);
                           } else {
                               Intake.setPower(1);
                               leftSweeper.setPower(-1);
                               rightSweeper.setPower(-1);
                           }


                           if (Math.abs(Arm.getCurrentPosition()) > 300){
                               Intake.setPower(1);
                               leftSweeper.setPower(-1);
                               rightSweeper.setPower(-1);
                           } else {
                               Intake.setPower(-1);
                               leftSweeper.setPower(-1);
                               rightSweeper.setPower(-1);
                           }
                           telemetry.addData("DistanceFromBot, KJs Bot", distanceFromBot);
                           telemetry.update();
                           if (gamepad1.dpad_left && distanceFromBot > 13) { //&& (Math.abs(((int)((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Arm.getCurrentPosition()) < 10) && Math.abs((int)((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength)) - extension.getCurrentPosition() < 10) {
                               if (TimerE.milliseconds() > 50) {
                                   distanceFromBot -= .5;
                                   TimerE.reset();
                               }
                           } else if (gamepad1.dpad_right && distanceFromBot < 26) { //&& (Math.abs(((int)((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Arm.getCurrentPosition()) < 10) && Math.abs((int)((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength)) - extension.getCurrentPosition() < 10) {
                               if (TimerE.milliseconds() > 50) {
                                   distanceFromBot += .5;
                                   TimerE.reset();
                               }
                           }
                           if (extension.getCurrentPosition() > -75 || extension.getCurrentPosition() < -700) {
                               Arm.setTargetPosition((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) *    tickPerDegree);
                               Arm.setPower(0.4);
                           }
                           if (armGoLeft && Arm.getCurrentPosition() < 0) {
                               if (Math.abs(((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Math.abs(Arm.getCurrentPosition())) < 100) {
                                   extension.setTargetPosition((int) ((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * -tickPerExtraLength));
                                   extension.setPower(0.75);
                               } else {
                                   extension.setTargetPosition(0);
                                   extension.setPower(1);
                               }
                           } else if (!armGoLeft && Arm.getCurrentPosition() > 0) {
                               if (Math.abs(((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Math.abs(Arm.getCurrentPosition())) < 100) {
                                   extension.setTargetPosition((int) ((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * -tickPerExtraLength));
                                   extension.setPower(0.75);
                               } else {
                                   extension.setTargetPosition(0);
                                   extension.setPower(1);
                               }
                           } else {
                               extension.setTargetPosition(0);
                               extension.setPower(1);
                           }
                       }
                    }*/

                } else if (!Palm && (Intake.getPower() != 0) && (armAtHome)) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.DEFAULT;
                }

                break;

        }
    }
}












