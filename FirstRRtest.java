package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name = "FirstRRtest", group = "")

public class FirstRRtest extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "Ae2mEyz/////AAABmQBmoTE94ki5quwzTT/OlIIeOueUfjuHL/5k1VNWN943meU2RmiXCJ9eX3rUR/2CkwguvbBU45e1SzrbTAwz3ZzJXc7XN1ObKk/7yPHQeulWpyJgpeZx+EqmZW6VE6yG4mNI1mshKI7vOgOtYxqdR8Yf7YwBPd4Ruy3NVK01BwBl1F8V/ndY26skaSlnWqpibCR3XIvVG0LXHTdNn/ftZyAFmCedLgLi1UtNhr2eXZdr6ioikyRYEe7qsWZPlnwVn5DaQoTcgccZV4bR1/PEvDLn7jn1YNwSimTC8glK+5gnNpO+X7BiZa5LcqtYEpvk/QNQda0Fd+wHQDXA8ojeMUagawtkQGJvpPpz9c6p4fad";
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia  = null;
    private VuforiaTrackables targets = null ;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private DcMotor Spinner, Arm;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    double currentX, currentY;

    private Servo Claw;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private OpenCvInternalCamera phoneCam;
    private ShippingElementDetector detector = new ShippingElementDetector();
    private String position;
    private float leftVote = 0;
    private float centerVote = 0;
    private float rightVote = 0;
    private String FinalDecision;
    private int armTopBehind = -2477;
    private int armMiddleBehind = -2702;
    private int armBottomFront = -250;
    private int armMiddleFront = -550;
    private int armTopFront = -900;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setPosition(0.1);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        Spinner = hardwareMap.get(DcMotor.class, "Spinner");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);


        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        position = detector.position;

        phoneCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);

        while (!isStarted()) {
            position = detector.position;

            switch (position) {
                case "Left":
                    leftVote += .01;
                    break;
                case "Center":
                    centerVote += .01;
                    break;
                case "Right":
                    rightVote += .01;
                    break;

            }
            if (leftVote > centerVote && leftVote > rightVote){
                FinalDecision = "Left";
            } else if (rightVote > leftVote && rightVote > centerVote){
                FinalDecision = "Right";
            } else {
                FinalDecision = "Center";
            }




            telemetry.addData("LeftVote",leftVote);
            telemetry.addData("CenterVote", centerVote);
            telemetry.addData("RightVote", rightVote);
            telemetry.update();
        }

        //    Spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //=======================================
        // Vuforia section
        //=======================================
      //  int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //=======================================
        // Roadrunner section
        //=======================================
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeRight(4)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(22)
                //.splineTo(new Vector2d(-59 , -60), Math.toRadians(180))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .splineTo(new Vector2d(-16 , -43), Math.toRadians(90))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(3)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(7)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(54)
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end().plus(new Pose2d(0,0,Math.toRadians(-35))))
                .forward(10)
                .build();
        Trajectory traj14 = drive.trajectoryBuilder(traj7.end())
                .back(9)
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj14.end().plus(new Pose2d(0,0,Math.toRadians(35))))
                .lineToConstantHeading(new Vector2d(43,-40))
               //  .strafeTo(new Vector2d(43,-40))
                //.splineTo(new Vector2d(43 , -40), Math.toRadians(0))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .back(45)
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(new Pose2d(-17, -43, Math.toRadians(90)))
                .forward(5)
                .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .back(1)
                .build();
        Trajectory traj13 = drive.trajectoryBuilder(traj12.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .forward(53)
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(traj5.end().plus(new Pose2d(0, 0, Math.toRadians(-30))))
                .forward(5)
                 .build();

/*
        while(!isStarted()){

            position = detector.position;

            switch (position) {
                case "Left":
                    leftVote += .01;
                    break;
                case "Center":
                    centerVote += .01;
                    break;
                case "Right":
                    rightVote += .01;
                    break;

            }
            if (leftVote > centerVote && leftVote > rightVote){
                FinalDecision = "Left";
            } else if (rightVote > leftVote && rightVote > centerVote){
                FinalDecision = "Right";
            } else {
                FinalDecision = "Center";
            }




            telemetry.addData("LeftVote",leftVote);
            telemetry.addData("CenterVote", centerVote);
            telemetry.addData("RightVote", rightVote);
            telemetry.update();

        }
*/
        waitForStart();




        telemetry.addData("I'm Here!", 2);
        telemetry.update();
        if(isStopRequested()) return;
        Claw.setPosition(0.1);
        Arm.setTargetPosition(-100);
        Arm.setPower(-1);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        Spinner.setPower(0.7);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }
        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT     = -9.0f * mmPerInch;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targets.activate();
        sleep(250);
        Spinner.setPower(0);

        //Arm.setTargetPosition(550);
        //Arm.setPower(1);
        drive.turn(Math.toRadians(-90));
        /*if (PositionLeft == true) {
            Arm.setTargetPosition(-219);
            Arm.setPower(-0.5);
        } else if (PositionMiddle == true) {
            Arm.setTargetPosition(-550);
            Arm.setPower(-0.5);
        } else if (PositionRight == true) {
            Arm.setTargetPosition(-900);
            Arm.setPower(-0.5);
        }*/
        switch (FinalDecision) {
            case "Left":
                Arm.setTargetPosition(-250);
                Arm.setPower(-0.5);
                break;
            case "Center":
                Arm.setTargetPosition(-550);
                Arm.setPower(-0.5);
                break;
            case "Right":
                Arm.setTargetPosition(-900);
                Arm.setPower(-0.5);
                break;

        }

        /*Arm.setTargetPosition(-900);
        Arm.setPower(-0.5);*/
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        Claw.setPosition(0.4);
        sleep(750);
        if (FinalDecision == "Left") {
            drive.followTrajectory(traj5);
            drive.turn(Math.toRadians(-30));
            Arm.setTargetPosition(0);
            Claw.setPosition(0.4);
            drive.followTrajectory(traj15);
            Claw.setPosition(0.25);
            sleep(250);
            Arm.setTargetPosition(-300);
            Arm.setPower(1);
            sleep(3000);
        }
        /*drive.followTrajectory(traj5);
        drive.turn(Math.toRadians(-90));
        Arm.setTargetPosition(-300);
        Arm.setPower(1);
        drive.followTrajectory(traj6);
        Arm.setTargetPosition(0);
        Arm.setPower(1);*/
        Claw.setPosition(0.4);
        drive.turn(Math.toRadians(-35));
        drive.followTrajectory(traj7);
        Claw.setPosition(0.25);
        sleep(500);
        Arm.setTargetPosition(-900);
        Arm.setPower(-0.7);
        drive.followTrajectory(traj14);
        drive.turn(Math.toRadians(35));
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        WalltargetTracking(false);
       drive.turn(Math.toRadians(90));
       telemetry.addData("CurrentX", currentX);
       telemetry.addData("CurrentY",currentY);
       telemetry.update();
     //  sleep(5000);
        Trajectory traj10 = drive.trajectoryBuilder(new Pose2d(currentX, currentY, Math.toRadians(90)))
               // .splineTo(new Vector2d(-17 , -43), Math.toRadians(90))
                .strafeTo(new Vector2d(-22,-35))
                .build();

        drive.followTrajectory(traj10);
        Claw.setPosition(0.4);
        sleep(1000);
        drive.followTrajectory(traj11);


        drive.followTrajectory(traj12);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(traj13);

       // Claw.setPosition(0.3);
     //   sleep(750);
/*
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);

       // drive.turn(Math.toRadians(-90));
        WalltargetTracking(false);
        drive.turn(Math.toRadians(180));
        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(currentX, currentY, Math.toRadians(180)))
                .splineTo(new Vector2d(-15 , -43), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj7);
      //  WalltargetTracking(false);
        sleep(1000);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(traj6);

   //     WalltargetTracking(false);

         */
        targets.deactivate();
    }

    private void MecanumFunction(double YL, double XL, double XR){
        leftFront.setPower(YL - XL + XR);
        rightFront.setPower(YL + XL - XR);
        leftRear.setPower(YL + XL + XR);
        rightRear.setPower(YL - XL - XR);
    }
    private void WalltargetTracking(boolean trueTracking){
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);
        while (opModeIsActive()){
            if (isStopRequested()){
                break;
            }
            targetVisible = false;


            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                if (trueTracking){
                    MecanumFunction(0,-0.1,0);
                    sleep(500);
                    MecanumFunction(0,0.1,0);
                    sleep(500);

                } else {
                    currentX = (translation.get(0) / mmPerInch);
                    currentY = (translation.get(1) / mmPerInch);
                    break;
                }

            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
    }
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
}

