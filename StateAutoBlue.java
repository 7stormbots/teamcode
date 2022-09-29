package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "StateAutoBlue", group = "")

public class StateAutoBlue extends LinearOpMode {
    private ColorSensor sensorColorRange_REV_ColorRangeSensor;
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
    private Servo Claw;
    public double grabberClose = 0.57;
    public double grabberOpen = 0.3;
    public double grabberIntakeOpen = 0.385;
    private int armTopBehind = -2420;
    private int armMiddleBehind = -2702;
    private int armBottomBehind = -3000;
    public double grabberSuperOpen = 0.2;
    private double intaking = -1;
    private double spitout = 1;
    double currentX, currentY;
    private double WristDown = 0.65;
    private double WristUp = 0;
    private boolean errorBlockNotFound;
    private DcMotorEx extension;
    private int Retracted = 0;
    private int Home = 0;
    private int partialExtension = -800;
    private RevBlinkinLedDriver blinkin;
    //  private int halfExtension =
    // private int Maximum = 442;

    //  private int halfExtension =
    // private int Maximum = 442;
    private int Maximum = -1950;
    //private int Maximum = -1950;
    private int TopLevelRight = 895;
    private int MiddleLevelRight = 740;
    private int BottomLevelRight = 572;
    private int TopLevelLeft = -895;
    private int MiddleLevelLeft = -680;
    private int BottomLevelLeft = -572;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private CRServo spinWheels;
    private Servo  tiltUp, swingWide;
  //  private OpenCvInternalCamera phoneCam;
    private ShippingElementDetectorBlueDepotv2 detector = new ShippingElementDetectorBlueDepotv2();
    private String position;
    private float leftVote = 0;
    private float centerVote = 0;
    private float rightVote = 0;
    private String FinalDecision;
    OpenCvWebcam webcam;
    private int wait = 500;
    private CRServo leftSweeper;
    private CRServo rightSweeper;
    @Override
    public void runOpMode(){

     // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        sensorColorRange_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        Claw.setPosition(grabberClose);
        sleep(500);
        extension.setPower(0.75);
        sleep(500);
        extension.setPower(0.25);
        sleep(1000);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(0);
        extension.setPower(1);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(1);
        extension.setTargetPosition(-40);
        extension.setPower(0.25);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSweeper = hardwareMap.get(CRServo.class, "leftSweeper");
        rightSweeper = hardwareMap.get(CRServo.class, "rightSweeper");
        swingWide = hardwareMap.get(Servo.class,"swingWide");
        tiltUp = hardwareMap.get(Servo.class, "tiltUp" );
        spinWheels = hardwareMap.get(CRServo.class, "spinWheels");
        //leftSweeper.setDirection(CRServo.Direction.REVERSE);
        //Arm.setDirection(DcMotorSimple.Direction.REVERSE);
       // phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
       // phoneCam.openCameraDevice();
   //     webcam.stopStreaming();
        webcam.setPipeline(detector);
        position = detector.position;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                          //   webcam.stopStreaming();
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }

                                         @Override
                                         public void onError(int errorCode) {

                                         }
                                     });

                //  phoneCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15.5, 66, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory hub = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0,50,Math.toRadians(-22)))
                .addDisplacementMarker(0, ()->{
                extension.setPower(1);
                extension.setTargetPosition(0);

                })
                .addDisplacementMarker(5, ()->{
                    if (FinalDecision == "Center"){
                        extension.setTargetPosition(-1690);
                        extension.setPower(1);
                    } else if (FinalDecision == "Left") {
                        extension.setTargetPosition(-1745);
                        extension.setPower(1);
                    } else {
                            extension.setTargetPosition(Maximum);
                            extension.setPower(1);
                        }

                })
                .addDisplacementMarker(20, ()->{
                    if (FinalDecision == "Right"){
                        Claw.setPosition(grabberSuperOpen);
                    } else {
                        Claw.setPosition(grabberOpen);
                    }

                })
                .build();

        Trajectory warehouse = drive.trajectoryBuilder(hub.end())
                .splineToConstantHeading(new Vector2d(9,67), Math.toRadians(40))
                .splineToSplineHeading(new Pose2d(15,70, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(46,70, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(0,() ->{
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(5,() ->{
                    Claw.setPosition(grabberClose);
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .addDisplacementMarker(17,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);

                })
                .addDisplacementMarker(19,() ->{
                    Claw.setPosition(grabberIntakeOpen);
                    Intake.setPower(intaking);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .build();
        Trajectory dropOff = drive.trajectoryBuilder(warehouse.end())
                .lineToLinearHeading(new Pose2d(36, 71, Math.toRadians(0)))
                //.splineToConstantHeading(new Vector2d(16, 68), Math.toRadians(-150))
                .splineToConstantHeading(new Vector2d(15, 65), Math.toRadians(-140))
                .splineToSplineHeading(new Pose2d(6, 50, Math.toRadians(-40)), Math.toRadians(-105))
                .addDisplacementMarker(0, ()->{
                    Intake.setPower(( 0.4 * spitout));
                    leftSweeper.setPower(-1);
                    rightSweeper.setPower(-1);

                })
                .addDisplacementMarker(5,() ->{
                    Intake.setPower(intaking);
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(13,() ->{
                    Arm.setTargetPosition(TopLevelRight);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(16, ()->{
                    Intake.setPower(spitout);
                })
                .addDisplacementMarker(22, ()->{
                    extension.setTargetPosition(Maximum);
                    extension.setPower(1);
                })
                .addTemporalMarker(2.9, ()->{
                    Claw.setPosition(grabberOpen);
                })
                .build();
        Trajectory warehouse1 = drive.trajectoryBuilder(dropOff.end())
                .splineToConstantHeading(new Vector2d(9,69), Math.toRadians(40))
                .splineToSplineHeading(new Pose2d(15,72, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(50,72, Math.toRadians(0)), Math.toRadians(0))

                .addDisplacementMarker(0,() ->{

                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(5,() ->{
                    Claw.setPosition(grabberClose);
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .addDisplacementMarker(17,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);

                })
                .addDisplacementMarker(19,() ->{
                    Claw.setPosition(grabberIntakeOpen);
                    Intake.setPower(intaking);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .build();
        Trajectory dropOff2 = drive.trajectoryBuilder(warehouse1.end())
                .lineToLinearHeading(new Pose2d(36, 71, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(15, 65), Math.toRadians(-140))
                .splineToSplineHeading(new Pose2d(6, 54, Math.toRadians(-30)), Math.toRadians(-103))
                .addDisplacementMarker(0, ()->{
                    Intake.setPower(( 0.4 * spitout));
                    leftSweeper.setPower(-1);
                    rightSweeper.setPower(-1);

                })
                .addDisplacementMarker(5,() ->{
                    Intake.setPower(intaking);
                })
                .addDisplacementMarker(10,() ->{
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })

                .addDisplacementMarker(13,() ->{
                    Arm.setTargetPosition(TopLevelRight);
                    Arm.setPower(1);

                })
                .addDisplacementMarker(22, ()->{
                    extension.setTargetPosition(Maximum);
                    extension.setPower(1);
                })
                .addTemporalMarker(2.9, ()->{
                    Claw.setPosition(grabberOpen);
                })
                .build();
        /*
        Trajectory warehouse2 = drive.trajectoryBuilder(dropOff2.end())
                .splineToConstantHeading(new Vector2d(9,69), Math.toRadians(40))
                .splineToSplineHeading(new Pose2d(15,72, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(54,72, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(0,() ->{

                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(5,() ->{
                    Claw.setPosition(grabberClose);
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .addDisplacementMarker(17,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);

                })
                .addDisplacementMarker(19,() ->{
                    Claw.setPosition(grabberIntakeOpen);
                    Intake.setPower(intaking);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .build();

         */
        Trajectory warehouse2 = drive.trajectoryBuilder(dropOff2.end())
                .splineToConstantHeading(new Vector2d(9,69), Math.toRadians(40))
                .splineToSplineHeading(new Pose2d(15,72, Math.toRadians(0)), Math.toRadians(0))
             //   .splineToSplineHeading(new Pose2d(40,72, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(56,65, Math.toRadians(0)), Math.toRadians(-20))
                .addDisplacementMarker(0,() ->{

                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(5,() ->{
                    Claw.setPosition(grabberClose);
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .addDisplacementMarker(17,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);

                })
                .addDisplacementMarker(19,() ->{
                    Claw.setPosition(grabberIntakeOpen);
                    Intake.setPower(intaking);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .build();

        Trajectory dropOff3 = drive.trajectoryBuilder(warehouse2.end())
                .lineToLinearHeading(new Pose2d(36, 72, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(15, 65), Math.toRadians(-150))//16, 70
                .splineToSplineHeading(new Pose2d(3, 54, Math.toRadians(-15)), Math.toRadians(-130))
                .addDisplacementMarker(0, ()->{
                    Intake.setPower(( 0.4 * spitout));
                    leftSweeper.setPower(-1);
                    rightSweeper.setPower(-1);
                })
                .addDisplacementMarker(5,() ->{
                    Intake.setPower(intaking);
                })
                .addDisplacementMarker(10,() ->{
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(13,() ->{
                    Arm.setTargetPosition(TopLevelRight);
                    Arm.setPower(1);

                })
                .addDisplacementMarker(22, ()->{
                    extension.setTargetPosition(Maximum + 100);
                    extension.setPower(1);
                })
                .addTemporalMarker(2.9, ()->{
                    Claw.setPosition(grabberOpen);
                })
                .build();
        Trajectory park = drive.trajectoryBuilder(dropOff3.end())
                .splineToConstantHeading(new Vector2d(9,69), Math.toRadians(40))
                .splineToSplineHeading(new Pose2d(15,72, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(54,72, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(0,() ->{
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(5,() ->{
                    Claw.setPosition(grabberClose);
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .addDisplacementMarker(17,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);

                })
                .addDisplacementMarker(19,() ->{
                    Claw.setPosition(grabberIntakeOpen);
                    Intake.setPower(intaking);
                    leftSweeper.setPower(1);
                    rightSweeper.setPower(1);
                })
                .build();


        ElapsedTime TimerA;
        ElapsedTime TimerB;
        ElapsedTime TimerC;
        TimerC = new ElapsedTime();
        TimerB = new ElapsedTime();
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
                telemetry.addData("RightTotal", detector.Righttotal);
                telemetry.addData("CenterTotal", detector.Centertotal);
                telemetry.update();
            }
            else {
                TimerA.reset();
                leftVote = 0;
                centerVote = 0;
                rightVote = 0;
            }
        }

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        swingWide.setPosition(0);
        tiltUp.setPosition(1);
        waitForStart();

//==================================================================================================================================


        if(isStopRequested()) return;
        OpenCVLogic();
        drive.followTrajectory(hub);
        drive.followTrajectory(warehouse);
        PickupBlock(100, 7, 1000);
        drive.followTrajectory(dropOff);
        drive.followTrajectory(warehouse1);
        PickupBlock(100, 7, 1000);
        drive.followTrajectory(dropOff2);
        drive.followTrajectory(warehouse2);
        PickupBlock(100, 7,1000);
        drive.followTrajectory(dropOff3);
        drive.followTrajectory(park);
/*
        OpenCVLogic();
        drive.followTrajectory(hub);
        drive.followTrajectory(warehouse1);

        Intake.setPower(.25 * intaking);
        Claw.setPosition(grabberClose);
        sleep(500);
        drive.followTrajectory(dropOff);
        drive.followTrajectory(warehouse2);
        Intake.setPower(.25 * intaking);
        Claw.setPosition(grabberClose);
        sleep(500);
        drive.followTrajectory(dropOff2);

 */

        webcam.stopStreaming();
    }

//==================================================================================================================================

    private void MecanumFunction(double YL, double XL, double XR){
        leftFront.setPower(0.8 * (YL - XL + XR));
        rightFront.setPower(0.8 * (YL + XL - XR));
        leftRear.setPower(0.8 * (YL + XL + XR));
        rightRear.setPower(0.8 * (YL - XL - XR));
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
                Arm.setTargetPosition(BottomLevelRight);
                Arm.setPower(1);
                break;
            case "Center":
                Arm.setTargetPosition(MiddleLevelRight);
                Arm.setPower(1);
                break;
            case "Right":
                Arm.setTargetPosition(TopLevelRight);
                Arm.setPower(1);
                break;
        }
    }
    private void PickupBlock(double timer, double distance, double timeout){
        ElapsedTime TimerH;
        ElapsedTime TimerG;
        TimerG = new ElapsedTime();
        TimerH = new ElapsedTime();
        TimerG.reset();
        TimerH.reset();
        while (TimerG.milliseconds() < timer){
            if (TimerH.milliseconds() > timeout){
                break;
            }
            if (((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > distance){
                TimerG.reset();
            }
        }

        Intake.setPower(0.8 * intaking);
        Claw.setPosition(grabberClose);
        sleep(300);
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

