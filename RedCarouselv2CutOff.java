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

@Autonomous(name = "RedCarousel - STOPS!", group = "")
@Disabled
public class RedCarouselv2CutOff extends LinearOpMode {

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

    private Servo Claw, Wrist;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private OpenCvInternalCamera phoneCam;
    private ShippingElementDetectorRedCarousel detector = new ShippingElementDetectorRedCarousel();
    private String position;
    private float leftVote = 0;
    private float centerVote = 0;
    private float rightVote = 0;
    private String FinalDecision;
    private int armTopBehind = -2450;
    private int armMiddleBehind = -2780;
    private int armBottomFront = -245;
    private int armMiddleFront = -530;
    private int armTopFront = -890;
    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        Claw = hardwareMap.get(Servo.class, "Claw");
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
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);


        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        position = detector.position;

        phoneCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37, -63, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        /*Trajectory strafe1 = drive.trajectoryBuilder(startPose)
                .strafeRight(4)
                .build();
        Trajectory forward1 = drive.trajectoryBuilder(strafe1.end())
                .forward(19)
                //.splineTo(new Vector2d(-59 , -60), Math.toRadians(180))
                .build();*/
        Trajectory carousel = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-57,-58,Math.toRadians(180)))
                .build();
        Trajectory straferight = drive.trajectoryBuilder(carousel.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                //   .spl-16 , -40), Math.toRadians(90))
                .strafeRight(4)
                .build();
        Trajectory lineToHub = drive.trajectoryBuilder(carousel.end())
                //   .splineTo(new Vector2d(-16 , -40), Math.toRadians(90))

                .lineToLinearHeading(new Pose2d(-12,-58,Math.toRadians(-90)))
                .addDisplacementMarker(1, ()->{
                    Wrist.setPosition(WristDown);
                })
                .addDisplacementMarker(18, ()->{
                    OpenCVLogic();

        })
                .build();
        Trajectory lineToHub2 = drive.trajectoryBuilder(carousel.end())
                //   .splineTo(new Vector2d(-16 , -40), Math.toRadians(90))

                .lineToLinearHeading(new Pose2d(-10,-44,Math.toRadians(90)))
                .addDisplacementMarker(10, ()->{
                    Wrist.setPosition(WristUp);
                })
                .addDisplacementMarker(20, ()->{
                    Arm.setTargetPosition(armBottomFront);
                    Arm.setPower(0.25);
                })
                .build();
        Trajectory forward2 = drive.trajectoryBuilder(lineToHub2.end())
                .forward(7)
                .build();
        Trajectory BackintoHub = drive.trajectoryBuilder(lineToHub.end())
                .back(7)

                .build();
        Trajectory backhub = drive.trajectoryBuilder(BackintoHub.end())
                .forward(7)
                .addDisplacementMarker(2,() -> {
                    Claw.setPosition(grabberClose);
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                })
                .build();
        Trajectory backhub2 = drive.trajectoryBuilder(forward2.end())
                .back(7)
                .addDisplacementMarker(5,() -> {
                    Claw.setPosition(grabberClose);
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                })
                .build();
        Trajectory goInDepot = drive.trajectoryBuilder(new Pose2d(-10,-52,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50,-40,Math.toRadians(0)))
                .addDisplacementMarker(43,() -> {
                    Arm.setTargetPosition(0);
                    Arm.setPower(0.5);
                })     .addDisplacementMarker(44,() -> {
                   // Intake.setPower(1);
                    Wrist.setPosition(WristDown);
                })
                .build();
        Trajectory goInDepotafterbackingup = drive.trajectoryBuilder((new Pose2d(-5,-32,Math.toRadians(0))))
                .lineToLinearHeading(new Pose2d(50,-34,Math.toRadians(0)))
                .addDisplacementMarker(43,() -> {
                    Arm.setTargetPosition(0);
                    Arm.setPower(0.5);
                })     .addDisplacementMarker(42,() -> {
                //    Intake.setPower(1);
                    Wrist.setPosition(WristDown);
                })
                .build();
        Trajectory goInBlockPile = drive.trajectoryBuilder(goInDepot.end().plus(new Pose2d(0,0,Math.toRadians(-60))))
                .forward(4)
                .build();
        Trajectory goInBlockPile2 = drive.trajectoryBuilder(goInBlockPile.end())
                .forward(3)
                .build();
        Trajectory goInBlockPile3 = drive.trajectoryBuilder(goInBlockPile2.end())
                .forward(3)
                .build();
        Trajectory backOutBlockPile = drive.trajectoryBuilder(goInBlockPile.end())
                .back(11)
                .build();
        Trajectory linetoexit = drive.trajectoryBuilder(goInBlockPile3.end())
                //   .splineTo(new Vector2d(-16 , -40), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(45,-40,Math.toRadians(0)))
                .addDisplacementMarker(0,() -> {
                    Arm.setTargetPosition(-970);
                    Arm.setPower(0.5);
                })
                .build();
        Trajectory backoutdepot = drive.trajectoryBuilder(linetoexit.end().plus(new Pose2d(0,0,Math.toRadians(0))))
                .back(36)
                .addDisplacementMarker(2,() -> {
                    Wrist.setPosition(WristUp);
                })    .addDisplacementMarker(7,() -> {
                })
                .build();

        Trajectory moveOver = drive.trajectoryBuilder(new Pose2d(-10.-44,Math.toRadians(-80)))
                .strafeRight(3)
                .build();
        Trajectory goInDepot2 = drive.trajectoryBuilder(new Pose2d(-18,-45,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50,-33, Math.toRadians(5)))
                .addDisplacementMarker(0,() -> {
                    Claw.setPosition(grabberClose);
                    Arm.setTargetPosition(-500);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(1,() -> {

                    Wrist.setPosition(WristUp);
                })
                .addDisplacementMarker(5,() -> {

                })
                .build();
        ElapsedTime TimerA;
        TimerA = new ElapsedTime();
        TimerA.reset();
        while (!isStarted()) {

            Wrist.setPosition(WristUp);
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
        Wrist.setPosition(WristDown);
        Wrist.setPosition(WristUp);
        Claw.setPosition(grabberClose);
        sleep(100);
        Arm.setTargetPosition(-100);
        Arm.setPower(-1);
        drive.followTrajectory(carousel);
        Spinner.setPower(0.85);
       InitializeVuforia();
       targets.activate();
        sleep(150);
        Spinner.setPower(0);
    //    drive.turn(Math.toRadians(-90));
     //   OpenCVLogic();
     //   drive.followTrajectory(straferight);
        if (FinalDecision == "Left"){
            drive.followTrajectory(lineToHub2);
            drive.followTrajectory(forward2);
            Claw.setPosition(grabberOpen);
            sleep(600);
        } else {
            drive.followTrajectory(lineToHub);
            drive.followTrajectory(BackintoHub);
            Claw.setPosition(grabberOpen);
            sleep(600);
        }

      //  drive.followTrajectory(forward2);

        if (FinalDecision == "Left"){
            drive.followTrajectory(backhub2);
            drive.turn(Math.toRadians(-90));
        } else {
            drive.followTrajectory(backhub);
            drive.turn(Math.toRadians(90));

        }

        targets.deactivate();
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

