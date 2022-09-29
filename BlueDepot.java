package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name = "BlueDepot", group = "")

public class BlueDepot extends LinearOpMode {

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
    public double grabberClose = 0.9;
    public double grabberOpen = 0.5;
    public double grabberHalfOpen = 0.75;
    private float phoneXRotate    = 0;
    double currentX, currentY;
    private Servo Claw;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private OpenCvInternalCamera phoneCam;
    private ShippingElementDetectorBlueDepot detector = new ShippingElementDetectorBlueDepot();
    private String position;
    private float leftVote = 0;
    private float centerVote = 0;
    private float rightVote = 0;
    private String FinalDecision;
    private int armTopBehind = -2477;
    private int armMiddleBehind = -2702;
    private int armBottomFront = -240;
    private int armMiddleFront = -530;
    private int armTopFront = -900;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setPosition(0.9);
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -63, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory strafe1 = drive.trajectoryBuilder(startPose)
                .strafeRight(4)
                .build();
        Trajectory line1 = drive.trajectoryBuilder(strafe1.end())
                .lineToLinearHeading(new Pose2d(-8 , -41,Math.toRadians(90)))
                .build();
        Trajectory forward2 = drive.trajectoryBuilder(line1.end())
                .forward(3)
                .build();
        Trajectory backhub = drive.trajectoryBuilder(line1.end())
                .back(13)
                .build();
        Trajectory driveintocargo = drive.trajectoryBuilder(backhub.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(55)
                .build();
        Trajectory drivetoblock = drive.trajectoryBuilder(driveintocargo.end())
                .forward(10)
                .build();
        Trajectory strafetogetout = drive.trajectoryBuilder(drivetoblock.end())
                .strafeLeft(11)
                .build();
        Trajectory backoverPVC = drive.trajectoryBuilder(strafetogetout.end())
                .back(43)
                .build();
        Trajectory Backupfromhub = drive.trajectoryBuilder(new Pose2d(-11,-38, Math.toRadians(90)))
                .back(15)
                .build();
        Trajectory goPark = drive.trajectoryBuilder(Backupfromhub.end().plus(new Pose2d(0,0, Math.toRadians(-90))))
                .forward(68)
                .build();



        waitForStart();

       //==================================================================================================================================

        InitializeVuforia();
        targets.activate();
        Claw.setPosition(grabberClose);
        Arm.setTargetPosition(-100);
        Arm.setPower(-1);
        drive.followTrajectory(strafe1);
        OpenCVLogic();
        drive.followTrajectory(line1);
        //drive.followTrajectory(forward2);
        Claw.setPosition(grabberOpen);
        sleep(750);
        drive.followTrajectory(backhub);
        Arm.setTargetPosition(armBottomFront);
        Arm.setPower(1);
        drive.turn(Math.toRadians(-90));
        Arm.setTargetPosition(armBottomFront);
        Arm.setPower(1);
        drive.followTrajectory(driveintocargo);
        Arm.setTargetPosition(0);
        Arm.setPower(1);
        Claw.setPosition(grabberHalfOpen);
        drive.followTrajectory(drivetoblock);
        Claw.setPosition(grabberClose);
        sleep(1000);
        Arm.setTargetPosition(armBottomFront);
        Arm.setPower(1);
        drive.followTrajectory(strafetogetout);
        drive.followTrajectory(backoverPVC);
    //    drive.followTrajectory(gotovuforia);
        WalltargetTracking(false);
        telemetry.addData("CurrentX", currentX);
        telemetry.addData("CurrentY",currentY);
        telemetry.update();
        Arm.setTargetPosition(armTopBehind);
        Arm.setPower(1);
        Trajectory vuforiasplinehub = drive.trajectoryBuilder(new Pose2d(currentX, currentY, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(0,-44, Math.toRadians(-67)))
                .build();
        Trajectory goPark2 = drive.trajectoryBuilder(vuforiasplinehub.end().plus(new Pose2d(0,0, Math.toRadians(67))))
                .forward(68)
                .build();
      //  drive.turn(Math.toRadians(90));
        drive.followTrajectory(vuforiasplinehub);
        sleep(500);
        Claw.setPosition(grabberHalfOpen);
        sleep(500);
        drive.turn(Math.toRadians(67));
        Arm.setTargetPosition(armBottomFront);
        Arm.setPower(0.5);
        drive.followTrajectory(goPark2);
   //     drive.followTrajectory(goPark);




        targets.deactivate();
    }
    //==================================================================================================================================


    private void MecanumFunction(double YL, double XL, double XR){
        leftFront.setPower(YL - XL + XR);
        rightFront.setPower(YL + XL - XR);
        leftRear.setPower(YL + XL + XR);
        rightRear.setPower(YL - XL - XR);
    }

    private void OpenCVLogic(){
        switch (FinalDecision) {
            case "Left":
                Arm.setTargetPosition(armBottomFront);
                Arm.setPower(-0.5);
                break;
            case "Center":
                Arm.setTargetPosition(armMiddleFront);
                Arm.setPower(-0.5);
                break;
            case "Right":
                Arm.setTargetPosition(armTopFront);
                Arm.setPower(-0.5);
                break;
        }
    }

    private void InitializeVuforia(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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


