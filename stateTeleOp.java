package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled

@TeleOp(name = "stateTeleOp", group = "")


public class stateTeleOp extends LinearOpMode {


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

    boolean GamepadbisPressed = false;




    @Override
    public void runOpMode(){

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive()) {
            ////////////////////////////////////////////////////
            // Make a MECANUM function to hide all this ugliness - Coach David
            ////////////////////////////////////////////////////


            leftFront.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)  - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            rightFront.setPower(-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y)- (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            leftRear.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            rightRear.setPower(-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));




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


        }
    }




    }











