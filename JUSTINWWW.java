package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "JUSTINWWW", group = "")
@Disabled

public class JUSTINWWW extends LinearOpMode {


    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private TouchSensor palm;
    private boolean GamepadBisPressed;
    private Servo Claw;
    private DcMotor Arm, Spinner;
    private boolean GamepadUpisPressed;
    private boolean GamepadDownisPressed;
    private boolean GamepadLeftisPressed;
    public double grabberClose = 0.65;
    public double grabberOpen = 0.15;
    public double grabberHalfOpen = 0.2;
    private boolean GamepadRightisPressed;
    private boolean GamepadYisPressed;
    private boolean GamepadAisPressed;
    private double Gamepad2Current ;
    private double Gamepad2Previous;
    private int armTopBehind = -2400;
    private int armMiddleBehind = -2702;
    private int armBottomFront = -240;
    private int armMiddleFront = -530;
    private int armTopFront = -890;
    private boolean GamepadxisPressed;


    @Override
    public void runOpMode(){

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        Claw = hardwareMap.get(Servo.class, "Claw");

        Spinner = hardwareMap.get(DcMotor.class, "Spinner");

        Spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        palm = hardwareMap.get(TouchSensor.class, "palm");

        waitForStart();
        while (opModeIsActive()) {
            leftFront.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)  - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            rightFront.setPower(-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y)- (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            leftRear.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            rightRear.setPower(-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));

            if (palm.isPressed()) {
                telemetry.addData("I'm pressed", 1);
            } else {
                telemetry.addData("I'm NOT pressed", 1);
            }
            telemetry.update();

            if (gamepad2.b){
                Claw.setPosition(grabberOpen);
            } else {
                Claw.setPosition(grabberClose);

            }
            if (gamepad2.y && !GamepadYisPressed) {
                Arm.setTargetPosition(Arm.getCurrentPosition() - 20);
                Arm.setPower(-1);
            } else if (gamepad2.a && !GamepadAisPressed) {
                Arm.setTargetPosition(Arm.getCurrentPosition() + 20);
                Arm.setPower(1);
            }
            GamepadAisPressed = gamepad2.a;
            GamepadYisPressed = gamepad2.y;
            if (gamepad2.dpad_up && !GamepadUpisPressed) {
                Arm.setTargetPosition(armMiddleBehind);
                Arm.setPower(-1);
            } else if (gamepad2.dpad_left && !GamepadLeftisPressed) {
                Arm.setTargetPosition(armBottomFront);
                Arm.setPower(-1);
            }
            else if (gamepad2.dpad_right && !GamepadRightisPressed) {
                Arm.setTargetPosition(armTopBehind);
                Arm.setPower(-1);
            }
            else if (gamepad2.dpad_down && !GamepadDownisPressed) {
                Arm.setTargetPosition(0);
                Arm.setPower(1);
            }
            if (gamepad2.x && !GamepadxisPressed){
                Arm.setTargetPosition(armTopBehind);
                Arm.setPower(-0.25);
            }
            GamepadxisPressed = gamepad2.x;
            GamepadDownisPressed = gamepad2.dpad_down;
            GamepadUpisPressed = gamepad2.dpad_up;
            GamepadRightisPressed = gamepad2.dpad_right;
            GamepadLeftisPressed = gamepad2.dpad_left;

            Spinner.setPower(gamepad2.left_trigger * .7);
        }
    }
}

