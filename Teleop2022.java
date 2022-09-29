package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robocol.RobocolDatagramSocket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Current;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name = "Teleop2022", group = "")

@Disabled
public class Teleop2022 extends LinearOpMode {


    private ColorSensor sensorColorRange_REV_ColorRangeSensor;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private DcMotor Intake;
    private TouchSensor palm;
    private boolean GamepadBisPressed;
    private boolean Break;
    private Servo Claw, Wrist;
    private DcMotor Arm, Spinner;
    private boolean GamepadUpisPressed;
    private boolean GamepadDownisPressed;
    private boolean GamepadLeftisPressed;
    public double grabberClose = 0.45;
    public double grabberOpen = 0.28;
    public double grabberHalfOpen = 0.35;
    private boolean GamepadRightisPressed;
    private boolean Gamepad1dpaddown = false;
    private boolean GamepadYisPressed;
    private boolean GamepadAisPressed;
    private boolean Clawhasbeenopened;
    private DcMotor Spinner2;
    private double Gamepad2Current ;
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
    ElapsedTime TimerC;
    ElapsedTime TimerD;
    private int currentArmPosition;
    private DigitalChannel RedLED2;
    private DigitalChannel GreenLED2;
    private DigitalChannel RedLED;
    private DigitalChannel GreenLED;
    private enum State {
        DEFAULT,
        INTAKE,
        GOINGUP,
        DROPPING,
    }
    private class IntakeStateMachine implements Runnable {
        public IntakeStateMachine() {
        }
        public void run() {
            while (opModeIsActive()) {
                doIntake();
            }
        }
    }



    @Override
    public void runOpMode(){

        TimerA = new ElapsedTime();

        TimerB = new ElapsedTime();

        TimerC = new ElapsedTime();

        TimerD = new ElapsedTime();


        /*RedLED = hardwareMap.get(DigitalChannel.class, "Red");
        GreenLED = hardwareMap.get(DigitalChannel.class, "Green");*/
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");
        sensorColorRange_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        Spinner = hardwareMap.get(DcMotor.class, "Spinner");
        Spinner2 = hardwareMap.get(DcMotor.class, "Spinner2");

        Spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spinner2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        // **** Arm homing is currently inactive.  This needs to be enabled to ensure that our arm setpoints are correct.  Probably want to do this before setting the arm mode (after the arm hardwaremap statement)
        /*
        currentArmPosition = 100;
        while (currentArmPosition - Arm.getCurrentPosition() > 0 || currentArmPosition - Arm.getCurrentPosition() < 0) {
            Arm.setPower(-0.2);
            sleep(50);
            currentArmPosition = Arm.getCurrentPosition();
        }

 */

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
            ////////////////////////////////////////////////////
            // Make a MECANUM function to hide all this ugliness - Coach David
            ////////////////////////////////////////////////////
            if (Palm){
                GreenLED.setState(true);
                GreenLED2.setState(true);
                RedLED2.setState(false);
                RedLED.setState(false);
            }
            else{
                GreenLED.setState(false);
                GreenLED2.setState(false);
                RedLED2.setState(true);
                RedLED.setState(true);



            }
            doIntake();
            telemetry.addData("Break", Break);
            telemetry.update();
            leftFront.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)  - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            rightFront.setPower(-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y)- (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            leftRear.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            rightRear.setPower(-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));

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

            Spinner.setPower(gamepad1.left_trigger * .7);
            Spinner2.setPower(gamepad1.left_trigger * .6);
        }
    }

    private void Homing() {
        Claw.setPosition(grabberClose);
        currentArmPosition = 100;
        Wrist.setPosition(WristDown);
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
//currentArmPosition - Arm.getCurrentPosition() > 0 || currentArmPosition - Arm.getCurrentPosition() < 0
    private void doIntake() {

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
                if ((Arm.getCurrentPosition() > -50) || (Arm.getCurrentPosition() < -500)){
                    Wrist.setPosition(WristUp);
                } else {
                 Wrist.setPosition(WristDown);
                    }
                NoCAP = 1;
        Clawhasbeenopened = false;
        Position = 2;
              //  Wrist.setPosition(.2);
                Intake.setPower(0);
                Claw.setPosition(grabberClose);
               Arm.setTargetPosition(-800);
                Arm.setPower(1);

                if (gamepad1.y == true) {
                    Break = false;
                    CurrentState = State.INTAKE;
            //        TimerB.reset();
                    Intakeon = true;
                    break;
                }else if (Palm || gamepad1.dpad_up) {
                    Break = false;

                    CurrentState = State.GOINGUP;
                   // (((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > 5)
                    break;
                } else if (Arm.getCurrentPosition() < armTopBehind) {
                    Break = false;
                    CurrentState = State.DROPPING;
                    break;
                } else if ((!(Palm || gamepad1.dpad_up)) && (!(gamepad1.y)) && !((Arm.getCurrentPosition() < -600)) && (Intake.getPower() == 0)){
                    Break = false;
                    CurrentState = State.DEFAULT;
                    break;
                }  else if (gamepad1.left_bumper && gamepad1.right_bumper && !gamepadBumperLeftisPressed && !gamepadBumperRightisPressed) {
                    Break = false;
                    Homing();
                } else if(gamepad1.left_stick_y >= 0.5 ){
                    Arm.setTargetPosition(Arm.getTargetPosition() - Increment);
                }
                else if (gamepad1.left_stick_y <= -0.5){
                    Arm.setTargetPosition(Arm.getTargetPosition() + Increment);
                } else if (Break) {
                    CurrentState = State.DEFAULT;
                }

                break;

            /*case BRAKE:
                Intake.setPower(0);
                Wrist.setPosition(WristUp);
                Claw.setPosition(grabberOpen);

                if (gamepad1.y) {
                    CurrentState = State.INTAKE;
                    break;
                }  else if (Arm.getCurrentPosition() < -600) {
                    CurrentState = State.DROPPING;
                    break;
                } else if ((!(Palm || gamepad1.dpad_up)) && (!(gamepad1.y)) && !((Arm.getCurrentPosition() < -600)) && (Intake.getPower() == 0)){
                    CurrentState = State.DEFAULT;
                    break;
                } else if ((Palm || gamepad1.dpad_up) && ArmGoingup) {

                    Claw.setPosition(grabberClose);
                    telemetry.addData("Armgoingup", ArmGoingup);
                    Intake.setPower(1);
                    if (TimerA.milliseconds() > 500) {
                        Arm.setPower(1);
                        Arm.setTargetPosition(armTopBehind);
                        if (TimerA.milliseconds() > 750) {
                            Wrist.setPosition(0);
                        }
                    }
                    Intakeon = false;
                    CurrentState = State.GOINGUP;
                    // (((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > 5)
                    break;
                }
                if (Arm.getCurrentPosition() < (armTopBehind + 10)){
                    ArmGoingup = false;
                    //  Wrist.setPosition(0);
                } else if (gamepad1.left_bumper && gamepad1.right_bumper && !gamepadBumperLeftisPressed && !gamepadBumperRightisPressed) {
                    Homing();
                } else if(gamepad1.left_stick_x >= 0.5 ){
                    Arm.setTargetPosition(Arm.getTargetPosition() - Increment);
                }
                else if (gamepad1.left_stick_x <= -0.5){
                    Arm.setTargetPosition(Arm.getTargetPosition() + Increment);
                } else if (gamepad1.x && GamepadxisPressed) {
                    CurrentState = State.BRAKE;
                }*/


            case INTAKE:

               // Claw.setPosition(grabberOpen);

                if (NoCAP == 1){
                    Intake.setPower(-1);
                } else {
                    Intake.setPower(0.01);
                }


                if ((Palm || gamepad1.dpad_up) && ArmGoingup) {
                    Intakeon = false;
                    ServoPosition = 0;
                    CurrentState = State.GOINGUP;
                    // (((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > 5)
                    break;
                } else if (Arm.getCurrentPosition() < -1000) {
                    ServoPosition = 0;
                    CurrentState = State.DROPPING;
                    break;
                } else if ((Break)||((!(Palm || gamepad1.dpad_up)) && (!(gamepad1.y)) && !((Arm.getCurrentPosition() < -600)) && (Intake.getPower() == 0))){
                    ServoPosition = 0;
                    CurrentState = State.DEFAULT;
                    break;
                } else if (Intakeon && !Palm) {
                    if (gamepad1.dpad_down && !GamepadDownisPressed){
                        NoCAP = NoCAP * -1;
                    }
                    GamepadDownisPressed = gamepad1.dpad_down;
                    if (NoCAP > 0){
                        Wrist.setPosition(WristDown);
                    } else {
                        Wrist.setPosition(WristUp);
                    }
                 //   telemetry.addData("ServoPosionwonfsoa", ServoPosition);
                    if (gamepad1.x){
                        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Arm.setTargetPosition(0);
                        Arm.setPower(1);
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Break = true;
                        break;
                    } else {
                        Break = false;
                    }
                    ServoPosition += .05;
                    if (ServoPosition > 0.5){
                        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        Arm.setPower(0.35);
                    }
                    if (Arm.getCurrentPosition() > -100){
                        Claw.setPosition(grabberOpen);
                    }
                    if (ServoPosition <= WristDown){
                        Wrist.setPosition(ServoPosition);
                    }

                    //  Wrist.setPosition(0);
                    telemetry.addData("fart",TimerB.milliseconds());
                    telemetry.addData("Palm", Palm);
                    telemetry.update();

                    Claw.setPosition(grabberOpen);

                    if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > 2.5)) {
                        TimerB.reset();

                        telemetry.addData("Palm?", "None");
                    }
                    else {

                        telemetry.addData("Palm?", "Yup");
                    }
                    if (TimerB.milliseconds() > 50) {
                        Palm = true;
                        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Arm.setTargetPosition(0);
                        Arm.setPower(1);
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmGoingup = true;
                        //if (NoCAP < 0){
                         //   Wrist.setPosition(WristDown);
                        //}
                    }
                    else {
                        Palm = false;
                    }
                    TimerA.reset();
                    break;

                } else if (gamepad1.left_bumper && gamepad1.right_bumper && !gamepadBumperLeftisPressed && !gamepadBumperRightisPressed) {
                    Homing();
                } else if(gamepad1.left_stick_y >= 0.5 ){
                    Arm.setTargetPosition(Arm.getTargetPosition() - Increment);
                }
                else if (gamepad1.left_stick_y <= -0.5){
                    Arm.setTargetPosition(Arm.getTargetPosition() + Increment);
                } else if (Break) {
                    CurrentState = State.DEFAULT;
                }




           // if (!(((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 5)) {

//            }

            case GOINGUP:

                if (gamepad1.y) {
                    CurrentState = State.INTAKE;
                    break;
                }  else if (Arm.getCurrentPosition() < -600) {
                    CurrentState = State.DROPPING;
                    break;
                } else if ((Break) || ((!(Palm || gamepad1.dpad_up)) && (!(gamepad1.y)) && !((Arm.getCurrentPosition() < -600)) && (Intake.getPower() == 0))){
                    CurrentState = State.DEFAULT;
                    break;
                } else if ((Palm || gamepad1.dpad_up) && ArmGoingup) {
                    if (gamepad1.x){
                        Break = true;
                        break;
                    } else {
                        Break = false;
                    }
                    Claw.setPosition(grabberClose);
                    telemetry.addData("Armgoingup", ArmGoingup);
                    Intake.setPower(1);
                    if (NoCAP > 0){
                        if (TimerA.milliseconds() > 400) {
                            if (gamepad1.a){
                                Arm.setTargetPosition(0);
                                Arm.setPower(1);
                                Wrist.setPosition(WristDown);
                            }else {
                                Arm.setPower(1);
                                Arm.setTargetPosition(armTopBehind);
                            }


                        }
                    } else {
                        if (TimerA.milliseconds() > 800) {
                            Arm.setPower(1);
                            Arm.setTargetPosition(armTopBehind);
                        }
                    }

                    if (NoCAP > 0){
                        if (Arm.getCurrentPosition() < -600){
                            Wrist.setPosition(0);
                        }
                    } else {
                        Wrist.setPosition(WristDown);
                    }

                    TimerD.reset();
                    Intakeon = false;
                    CurrentState = State.GOINGUP;
                    // (((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > 5)
                    break;
                }
                if (Arm.getCurrentPosition() < (armTopBehind + 10)){
                        ArmGoingup = false;
                //  Wrist.setPosition(0);
                } else if (gamepad1.left_bumper && gamepad1.right_bumper && !gamepadBumperLeftisPressed && !gamepadBumperRightisPressed) {
                    Homing();
                } else if(gamepad1.left_stick_y >= 0.5  ){
                    Arm.setTargetPosition(Arm.getTargetPosition() - Increment);
                }
                else if (gamepad1.left_stick_y <= -0.5){
                    Arm.setTargetPosition(Arm.getTargetPosition() + Increment);
                } else if (gamepad1.x && GamepadxisPressed) {
                    CurrentState = State.DEFAULT;
                }



                break;


            case DROPPING:
                Wrist.setPosition(WristUp);
                ArmGoingup = false;
                if (gamepad1.y) {
                    CurrentState = State.INTAKE;
                    Intakeon = true;
                    break;
                } else if ((Palm || gamepad1.dpad_up) && ArmGoingup) {

                    CurrentState = State.GOINGUP;
                    break;
                } else if ((Break)||((((!Palm && Clawhasbeenopened) || gamepad1.dpad_up)) && (!(gamepad1.y)) && ((Arm.getCurrentPosition() > -600)) && (Intake.getPower() == 0))){

                    CurrentState = State.DEFAULT;
                    break;
                } //else if (Arm.getCurrentPosition() < -600) {
                if (gamepad1.x){
                    Break = true;
                    break;
                } else {
                    Break = false;
                }
                    Intake.setPower(0);
                    if ((gamepad1.dpad_left && Position > 0) && !GamepadLeftisPressed){
                        Position -= 1;
                    }
                    GamepadLeftisPressed = gamepad1.dpad_left;
                    if ((gamepad1.dpad_right && Position < 3) && !GamepadRightisPressed){
                        Position += 1;
                    }
                    ArmGoingup = false;
                    GamepadRightisPressed = gamepad1.dpad_right;
                    if (TimerB.milliseconds() < 200){

                        if (Position == 2) {
                            Arm.setTargetPosition(armTopBehind);
                        } else if (Position == 1) {
                            Arm.setTargetPosition(armMiddleBehind);
                        } else if (Position == 0){
                            Arm.setTargetPosition(armBottomBehind);
                        } else if (Position == 3){
                            Arm.setTargetPosition(-2221);
                        }
                        Arm.setPower(1);

                    }

                    if (gamepad1.b){
                        Claw.setPosition(grabberOpen);
                        if (TimerD.milliseconds() > 300){
                            Clawhasbeenopened = true;
                        }

                    } else {
                        TimerD.reset();
                    }
                    telemetry.addData("Clawhasbeenopened?", Clawhasbeenopened);
                    telemetry.update();

                    if ((((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 5)) {
                        TimerB.reset();
                    }
                    if (TimerB.milliseconds() > 200){
                        Palm = false;
                        if (Clawhasbeenopened){
                            Arm.setTargetPosition(-800);
                            Arm.setPower(1);
                            //  Arm.setTargetPosition(armBottomFront);
                        //    Claw.setPosition(grabberClose);
                            Wrist.setPosition(1);
                        }

                    }  else if (gamepad1.left_bumper && gamepad1.right_bumper && gamepadBumperLeftisPressed && gamepadBumperRightisPressed) {
                        Homing();
                    } else if(gamepad1.left_stick_y >= 0.5 ){
                        Arm.setTargetPosition(Arm.getTargetPosition() - Increment);
                    }
                    else if (gamepad1.left_stick_y <= -0.5){
                        Arm.setTargetPosition(Arm.getTargetPosition() + Increment);
                    }
                    if (Break || (((Palm == false)) && Clawhasbeenopened)) {
                        CurrentState = State.DEFAULT;
                        break;
                    }


                    break;

                }

        }
    }











