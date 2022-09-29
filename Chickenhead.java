package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "Chicken Head", group = "")
@Disabled

public class Chickenhead extends LinearOpMode {


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
    private float targetArmAngle = 30;
    private boolean gamepadlefttrigger;
    private int armBottomFront = -240;
    private int armMiddleFront = -530;
    private int armTopFront = -890;
    private boolean GamepadxisPressed;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
   // Acceleration gravity;

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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        //    composeTelemetry();

        // Set up our telemetry dashboard

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        palm = hardwareMap.get(TouchSensor.class, "palm");

        waitForStart();
        while (opModeIsActive()) {
            leftFront.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) - (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x)  - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            rightFront.setPower(-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y)- (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            leftRear.setPower((gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            rightRear.setPower(-(gamepad2.left_stick_y * gamepad2.left_stick_y * gamepad2.left_stick_y) + (gamepad2.left_stick_x * gamepad2.left_stick_x * gamepad2.left_stick_x) - (gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x));
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            /*
            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });
            telemetry.update();

             */
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            /*

            if (palm.isPressed()) {
                telemetry.addData("I'm pressed", 1);
            } else {
                telemetry.addData("I'm NOT pressed", 1);
            }
            telemetry.update();


             */
        //    Arm.setTargetPosition(-(Math.round((targetArmAngle * 11) + Math.round(angles.thirdAngle * 11.07))));
            Arm.setPower(((-(Math.round((targetArmAngle * 11) + Math.round(angles.thirdAngle * 11.07)))) - Arm.getCurrentPosition()) * 0.01);
            telemetry.addData("fart", (((-(Math.round((targetArmAngle * 11) + Math.round(angles.thirdAngle * 11.07)))) - Arm.getCurrentPosition())) * 0.01);
            telemetry.update();
            if (gamepad1.b){
                Claw.setPosition(grabberOpen);
            } else {
                Claw.setPosition(grabberClose);

            }

            if (gamepad1.y && !GamepadYisPressed) {
                Arm.setTargetPosition(Arm.getCurrentPosition() - 20);
                Arm.setPower(-1);
            } else if (gamepad1.a && !GamepadAisPressed) {
                Arm.setTargetPosition(Arm.getCurrentPosition() + 20);
                Arm.setPower(1);
            }
            GamepadAisPressed = gamepad1.a;
            GamepadYisPressed = gamepad1.y;
            if (gamepad1.dpad_up && !GamepadUpisPressed) {
              targetArmAngle = 100;
            } else if (gamepad1.dpad_left && !GamepadLeftisPressed) {
            targetArmAngle = 20;
            }
            else if (gamepad1.dpad_right && !GamepadRightisPressed) {
               targetArmAngle = 80;
            }
            else if (gamepad1.dpad_down && !GamepadDownisPressed) {
                targetArmAngle = 0;
            }
            if (gamepad1.x && !GamepadxisPressed){
                Arm.setTargetPosition(armTopBehind);
                Arm.setPower(-0.25);
            }
            if (gamepad1.left_bumper && !gamepadlefttrigger){
                Arm.setTargetPosition(-1111);
                Arm.setPower(-0.75);
            }
            gamepadlefttrigger = gamepad1.left_bumper;
            GamepadxisPressed = gamepad1.x;
            GamepadDownisPressed = gamepad1.dpad_down;
            GamepadUpisPressed = gamepad1.dpad_up;
            GamepadRightisPressed = gamepad1.dpad_right;
            GamepadLeftisPressed = gamepad1.dpad_left;

            Spinner.setPower(gamepad1.left_trigger * .7);
        }
    }
    /*
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

     */
}
