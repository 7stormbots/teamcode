package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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


@TeleOp(name = "TeleopSmootingFunction", group = "")


public class TeleopSmoothingFunction extends LinearOpMode {


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
    private boolean wegood;
    private boolean GamepadLeftisPressed;
    private boolean gamepad2leftdpadisPressed;
    private boolean gamepad2rightdpadisPressed;
    public double grabberClose = 0;
    public double grabberOpen = 0.55;
    public double grabberIntakeOpen = 0.525;
    public double grabberSuperOpen = 0.7;
    public boolean slowmode = true;
    private boolean GamepadRightisPressed;
    private boolean sharedHub = false;
    private boolean Gamepad1dpaddown = false;
    private boolean GamepadYisPressed;
    ElapsedTime TimerE;
    private boolean GamepadAisPressed;
    private boolean Clawhasbeenopened;
    //private DcMotor Spinner;
    private int tickPerDegree = 10;
    private double tickPerExtraLength = 442 / 9.875;
    private double height = 11;
    private double distanceFromBot = 15;
    private double Gamepad2Current;
    private double Gamepad2Previous;
    private int armTopBehind = -2420;
    private int armMiddleBehind = -2702;
    private int armBottomBehind = -3000;
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
    ElapsedTime TimerB;
    private DcMotor Spinner;
    ElapsedTime TimerC;
    ElapsedTime TimerD;
    private int currentArmPosition;
    private DigitalChannel RedLED2;
    private DigitalChannel GreenLED2;
    private DigitalChannel RedLED;
    private DigitalChannel GreenLED;
    private DcMotorEx extension;
    private int Retracted = 0;
    private int Home = 0;
    private int partialExtension = 174;
    //  private int halfExtension =
    // private int Maximum = 442;
    private int Maximum = 200;
    private double FilteredXR, LastFilteredXR, FilteredXL, LastFilteredXL, FilteredYL, LastFilteredYL;
    private int TopLevelLeft = -870;
    private int TopLevelRight = 870;

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


        /*RedLED = hardwareMap.get(DigitalChannel.class, "Red");
        GreenLED = hardwareMap.get(DigitalChannel.class, "Green");*/
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        Claw = hardwareMap.get(Servo.class, "Claw");
        Blinkers = hardwareMap.get(Servo.class, "Blinkers");
        sensorColorRange_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        //Spinner = hardwareMap.get(DcMotor.class, "Spinner");
        extension = hardwareMap.get(DcMotorEx.class, "extension");

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

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        palm = hardwareMap.get(TouchSensor.class, "palm");
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        extension.setPower(-0.5);
        sleep(2000);

        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(0);
        extension.setPower(1);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        RedLED.setState(true);
        RedLED2.setState(true);
        // Homing();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.dpad_up && !GamepadUpisPressed){
                if (slowmode) {
                    slowmode = false;
                } else {
                    slowmode = true;
                }
            }

            MecanumFunction(FilteredYL, FilteredXL, FilteredXR);

            GamepadUpisPressed = gamepad2.dpad_up;
            ////////////////////////////////////////////////////
            // Make a MECANUM function to hide all this ugliness - Coach David
            ////////////////////////////////////////////////////
            if (armGoLeft) {
                Blinkers.setPosition(0.7);
            } else {
                Blinkers.setPosition(0.25);
            }
            if (Palm) {
                GreenLED.setState(true);
                GreenLED2.setState(true);
                RedLED2.setState(false);
                RedLED.setState(false);
            } else {
                GreenLED.setState(false);
                GreenLED2.setState(false);
                RedLED2.setState(true);
                RedLED.setState(true);


            }
            ultimateState();
            telemetry.addData("Break", Break);
            telemetry.update();
            if (slowmode) {
                leftFront.setPower((0.46 * ((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)) - (0.35 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                rightFront.setPower((0.46 * (-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)) - (0.35 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                leftRear.setPower((0.46* ((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)) - (0.35 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                rightRear.setPower((0.46 * (-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)) - (0.35 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));

            } else {
                leftFront.setPower(1 * ((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (0.75 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                rightFront.setPower(1 * (-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (0.75 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                leftRear.setPower(1 * ((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (0.75 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));
                rightRear.setPower(1 * (-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (0.75 * (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x))));

            }

            if (palm.isPressed()) {
                telemetry.addData("I'm pressed", 1);
            } else {
                telemetry.addData("I'm NOT pressed", 1);
            }
            telemetry.addData("CurrentState:", CurrentState);
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
            telemetry.addData("Andy NUTZ", TimerC.milliseconds());
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
        FilteredXL = 0.1 * gamepad2.left_stick_x + 0.9 * LastFilteredXL;
        FilteredXR = 0.1 * gamepad2.right_stick_x + 0.9 * LastFilteredXR;

        LastFilteredYL = FilteredYL;
        LastFilteredXL = FilteredXL;
        LastFilteredXR = FilteredXR;
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
                if (Intake.getPower() >= 0) {
                    if (gamepad2.right_bumper && !gamepad2leftdpadisPressed) {
                        armGoLeft = false;
                    } else if (gamepad2.left_bumper && !gamepad2rightdpadisPressed) {
                        armGoLeft = true;
                    }
                    gamepad2leftdpadisPressed = gamepad2.left_bumper;
                    gamepad2rightdpadisPressed = gamepad2.right_bumper;
                    extension.setTargetPosition(Retracted);
                    extension.setPower(1);
                    if (extension.getCurrentPosition() < 20) {
                        Arm.setTargetPosition(Home);
                        if ((Math.abs(Arm.getCurrentPosition()) < 200)) {
                            Arm.setPower(0.25);
                        } else if ((Math.abs(Arm.getCurrentPosition()) < 20)) {
                            Arm.setPower(1);
                        } else {
                            Arm.setPower(1);

                        }
                    }

                    Spinner.setPower(0.75 * (gamepad2.right_trigger - gamepad2.left_trigger));

                    Claw.setPosition(grabberOpen);
                    if (extension.getCurrentPosition() < 20){
                        if (gamepad2.y) {
                            Intake.setPower(-1);
                        } else {
                            Intake.setPower(1);
                        }
                    }

                } else if (!Palm) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.DROP;
                }
                break;

            case INTAKE:
                if (!Palm && (Intake.getPower() != 0)) {
                    if (gamepad2.dpad_down) {
                        Intake.setPower(0);
                    } else {
                        if (extension.getCurrentPosition() > 160){
                            Intake.setPower(-1);
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

                    if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > 4)) {
                        TimerB.reset();

                        telemetry.addData("Palm?", "None");
                    }

                    Spinner.setPower(0.75 * (gamepad2.right_trigger - gamepad2.left_trigger));
                    if (TimerB.milliseconds() >= 500) {
                        Palm = true;
                        Intake.setPower(0);
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
                    if ((partialExtension - extension.getCurrentPosition() <= 20)) {

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
                    if (!sharedHub) {


                        if (gamepad2.x) {
                            sharedHub = true;
                        } else if (gamepad2.b) {
                            sharedHub = false;
                        }
                        if (gamepad2.a) {
                            Claw.setPosition(grabberSuperOpen);
                        } else {
                            Claw.setPosition(grabberClose);
                        }
                        if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 4)) {
                            TimerB.reset();

                            telemetry.addData("Palm?", "None");
                        }

                        if (gamepad2.dpad_down) {
                            Palm = false;
                            Intake.setPower(0);
                        } else {
                            if (extension.getCurrentPosition() < 50 || extension.getCurrentPosition() > 210){
                                Intake.setPower(1);
                            }

                        }
                        if (Arm.getCurrentPosition() < -600 || Arm.getCurrentPosition() > 600) {
                            if (!armGoLeft && Arm.getCurrentPosition() > -600) {
                                extension.setTargetPosition(Maximum);
                                extension.setPower(1);
                            } else if (armGoLeft && Arm.getCurrentPosition() < 600) {
                                extension.setTargetPosition(Maximum);
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
                            if (extension.getCurrentPosition() <= 20) {
                                Arm.setTargetPosition(TopLevelLeft);
                                Arm.setPower(1);

                            }
                        } else {
                            telemetry.addData("Currently in regular drop right mode", 1);
                            telemetry.update();

                            if (extension.getCurrentPosition() <= 20) {
                                Arm.setTargetPosition(TopLevelRight);
                                Arm.setPower(1);


                            }
                        }

                    } else if (sharedHub) {

                       if (extension.getCurrentPosition() < 50){
                            Intake.setPower(1);
                        }
                        if (gamepad2.x) {
                            sharedHub = true;
                        } else if (gamepad2.b) {
                            sharedHub = false;
                        }
                        telemetry.addData("Currently in shared hub left mode", 1);
                        telemetry.update();
                        if (gamepad2.a) {
                            Claw.setPosition(grabberSuperOpen);
                        } else {
                            Claw.setPosition(grabberClose);
                        }
                        if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 4)) {
                            TimerB.reset();

                            telemetry.addData("Palm?", "None");
                        }

                        if (TimerB.milliseconds() >= 250 || gamepad2.dpad_down) {
                            Palm = false;
                            Intake.setPower(0);
                        } else {
                            if (extension.getCurrentPosition() < 50 || extension.getCurrentPosition() > 210){
                                Intake.setPower(1);
                            }
                        }

                        if (gamepad2.right_bumper && !gamepad2leftdpadisPressed) {
                            armGoLeft = false;
                        } else if (gamepad2.left_bumper && !gamepad2rightdpadisPressed) {
                            armGoLeft = true;
                        }
                        gamepad2leftdpadisPressed = gamepad2.left_bumper;
                        gamepad2rightdpadisPressed = gamepad2.right_bumper;
                        if (armGoLeft) {
                            if (gamepad2.dpad_left && distanceFromBot > 13) { //&& (Math.abs(((int)((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Arm.getCurrentPosition()) < 10) && Math.abs((int)((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength)) - extension.getCurrentPosition() < 10) {
                                if (TimerE.milliseconds() > 50) {
                                    distanceFromBot -= .5;
                                    TimerE.reset();
                                }
                            } else if (gamepad2.dpad_right && distanceFromBot < 20) { //&& (Math.abs(((int)((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Arm.getCurrentPosition()) < 10) && Math.abs((int)((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength)) - extension.getCurrentPosition() < 10) {
                                if (TimerE.milliseconds() > 50) {
                                    distanceFromBot += .5;
                                    TimerE.reset();
                                }
                            }
                            if (extension.getCurrentPosition() < 50 || extension.getCurrentPosition() > 190) {
                                Arm.setTargetPosition((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * -tickPerDegree);
                                Arm.setPower(0.4);
                            }



                            if (armGoLeft && Arm.getCurrentPosition() < 0) {
                                if (Math.abs(((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Math.abs(Arm.getCurrentPosition())) < 100) {
                                    extension.setTargetPosition((int) ((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength));
                                    extension.setPower(0.75);
                                } else {
                                    extension.setTargetPosition(0);
                                    extension.setPower(1);
                                }
                            } else if (!armGoLeft && Arm.getCurrentPosition() > 0) {
                                if (Math.abs(((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Math.abs(Arm.getCurrentPosition())) < 100) {
                                    extension.setTargetPosition((int) ((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength));
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
                            if (extension.getCurrentPosition() < 50){
                                Intake.setPower(1);
                            }
                            telemetry.addData("DistanceFromBot", distanceFromBot);
                            telemetry.update();
                            if (gamepad2.dpad_left && distanceFromBot > 13) { //&& (Math.abs(((int)((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Arm.getCurrentPosition()) < 10) && Math.abs((int)((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength)) - extension.getCurrentPosition() < 10) {
                                if (TimerE.milliseconds() > 50) {
                                    distanceFromBot -= .5;
                                    TimerE.reset();
                                }
                            } else if (gamepad2.dpad_right && distanceFromBot < 20) { //&& (Math.abs(((int)((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Arm.getCurrentPosition()) < 10) && Math.abs((int)((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength)) - extension.getCurrentPosition() < 10) {
                                if (TimerE.milliseconds() > 50) {
                                    distanceFromBot += .5;
                                    TimerE.reset();
                                }
                            }

                            if (extension.getCurrentPosition() < 50 || extension.getCurrentPosition() > 190) {
                                Arm.setTargetPosition((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * -tickPerDegree);
                                Arm.setPower(0.4);
                            }

                            if (armGoLeft && Arm.getCurrentPosition() < 0) {
                                if (Math.abs(((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Math.abs(Arm.getCurrentPosition())) < 100) {
                                    extension.setTargetPosition((int) ((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength));
                                    extension.setPower(0.75);
                                } else {
                                    extension.setTargetPosition(0);
                                    extension.setPower(1);
                                }
                            } else if (!armGoLeft && Arm.getCurrentPosition() > 0) {
                                if (Math.abs(((int) ((Math.atan(distanceFromBot / height) / Math.PI * 180)) * tickPerDegree) - Math.abs(Arm.getCurrentPosition())) < 100) {
                                    extension.setTargetPosition((int) ((Math.sqrt((height * height) + (distanceFromBot * distanceFromBot)) - 13.25) * tickPerExtraLength));
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
                    }

                } else if (!Palm) {
                    CurrentState = State.INTAKE;
                } else {
                    CurrentState = State.DEFAULT;
                }

                break;

        }
    }
}












