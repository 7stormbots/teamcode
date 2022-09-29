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
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "stateRED2", group = "")
@Disabled
public class stateRED2 extends LinearOpMode {

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
    private DcMotor Intake;
    public double grabberClose = 0.5;
    public double grabberOpen = 0.28;
    public double grabberHalfOpen = 0.35;
    private double intaking = -1;
    private double spitout = 1;
    double currentX, currentY;
    private double WristDown = 0.65;
    private double WristUp = 0;


    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private OpenCvInternalCamera phoneCam;
    private ShippingElementDetectorRedDepot detector = new ShippingElementDetectorRedDepot();
    private String position;
    private float leftVote = 0;
    private float centerVote = 0;
    private float rightVote = 0;
    private String FinalDecision;
    private int armTopBehind = -2450;
    private int armMiddleBehind = -2720;
    private int armBottomFront = -245;
    private int armMiddleFront = -530;
    private int armTopFront = -890;
    private int wait = 500;
    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /*Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setPosition(grabberClose);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        Spinner = hardwareMap.get(DcMotor.class, "Spinner");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);

        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);*/


        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        position = detector.position;

        phoneCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -63, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory hub = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-6,-63,Math.toRadians(0)))
                .build();
        Trajectory warehouse = drive.trajectoryBuilder(hub.end())
                .splineToSplineHeading(new Pose2d(24,-63, Math.toRadians(0)), Math.toRadians(0))
                .forward(28)
             //   .lineToLinearHeading(new Pose2d(52,-63,Math.toRadians(0)))
                .build();
        Trajectory hub1 = drive.trajectoryBuilder(warehouse.end())
               // .lineToLinearHeading(new Pose2d(-6,-63,Math.toRadians(0)))
                .back(28)
                .splineToSplineHeading(new Pose2d(6,-36, Math.toRadians(45)), Math.toRadians(0))
                .build();
        Trajectory warehouse1 = drive.trajectoryBuilder(hub1.end())
                .lineToLinearHeading(new Pose2d(52,-63,Math.toRadians(0)))
                .build();
        Trajectory hub2 = drive.trajectoryBuilder(warehouse1.end())
                .lineToLinearHeading(new Pose2d(-6,-63,Math.toRadians(0)))
                .build();
        Trajectory warehouse2 = drive.trajectoryBuilder(hub2.end())
                .lineToLinearHeading(new Pose2d(52,-63,Math.toRadians(0)))
                .build();
        Trajectory hub3 = drive.trajectoryBuilder(warehouse2.end())
                .lineToLinearHeading(new Pose2d(-6,-63,Math.toRadians(0)))
                .build();
        Trajectory warehouse3 = drive.trajectoryBuilder(hub3.end())
                .lineToLinearHeading(new Pose2d(52,-63,Math.toRadians(0)))
                .build();
        Trajectory hub4 = drive.trajectoryBuilder(warehouse3.end())
                .lineToLinearHeading(new Pose2d(-6,-63,Math.toRadians(0)))
                .build();
        Trajectory warehouse4 = drive.trajectoryBuilder(hub4.end())
                .lineToLinearHeading(new Pose2d(52,-63,Math.toRadians(0)))
                .build();
        Trajectory hub5 = drive.trajectoryBuilder(warehouse4.end())
                .lineToLinearHeading(new Pose2d(-6,-63,Math.toRadians(0)))
                .build();
        Trajectory warehouse5 = drive.trajectoryBuilder(hub5.end())
                .lineToLinearHeading(new Pose2d(52,-63,Math.toRadians(0)))
                .build();
        Trajectory hub6 = drive.trajectoryBuilder(warehouse5.end())
                .lineToLinearHeading(new Pose2d(-6,-63,Math.toRadians(0)))
                .build();
        Trajectory warehouse6 = drive.trajectoryBuilder(hub6.end())
                .lineToLinearHeading(new Pose2d(52,-63,Math.toRadians(0)))
                .build();
        ElapsedTime TimerA;
        TimerA = new ElapsedTime();
        TimerA.reset();
        while (!isStarted()) {


            if (TimerA.milliseconds() < 5000) {
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
                if (leftVote > centerVote && leftVote > rightVote) {
                    FinalDecision = "Left";
                } else if (rightVote > leftVote && rightVote > centerVote) {
                    FinalDecision = "Right";
                } else {
                    FinalDecision = "Center";
                }


                telemetry.addData("LeftVote", leftVote);
                telemetry.addData("CenterVote", centerVote);
                telemetry.addData("RightVote", rightVote);
                telemetry.update();
            }
            else {
                TimerA.reset();
                leftVote = 0;
                centerVote = 0;
                rightVote = 0;
            }
        }







        waitForStart();

//==================================================================================================================================


        if(isStopRequested()) return;
        drive.followTrajectory(hub);
        sleep(wait);
        drive.followTrajectory(warehouse);
        sleep(wait);
        drive.followTrajectory(hub1);
        sleep(wait);
        drive.followTrajectory(warehouse1);
        sleep(wait);
        drive.followTrajectory(hub2);
        sleep(wait);
        drive.followTrajectory(warehouse2);
        sleep(wait);
        drive.followTrajectory(hub3);
        sleep(wait);
        drive.followTrajectory(warehouse3);
        sleep(wait);
        drive.followTrajectory(hub4);
        sleep(wait);
        drive.followTrajectory(warehouse4);
        sleep(wait);
        drive.followTrajectory(hub5);
        sleep(wait);
        drive.followTrajectory(warehouse5);
        sleep(wait);







































































       // targets.deactivate();
    }

//==================================================================================================================================

    private void MecanumFunction(double YL, double XL, double XR){
        leftFront.setPower(YL - XL + XR);
        rightFront.setPower(YL + XL - XR);
        leftRear.setPower(YL + XL + XR);
        rightRear.setPower(YL - XL - XR);
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
        final float CAMERA_LEFT_DISPLACEMENT     = 8.0f * mmPerInch;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }
    private void OpenCVLogic(){
        switch (FinalDecision) {
            case "Left":
                Arm.setTargetPosition(-215);
                Arm.setPower(-0.5);
                break;
            case "Center":
                Arm.setTargetPosition(armMiddleBehind);
                Arm.setPower(-1);
                break;
            case "Right":
                Arm.setTargetPosition(-2500);
                Arm.setPower(-0.75);
                break;
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

