package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import java.util.Vector;

@Autonomous(name = "Justin", group = "")

public class stateREDv4 extends LinearOpMode {
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
    public double grabberClose = 0;
    public double grabberOpen = 0.55;
    public double grabberIntakeOpen = 0.525;
    public double grabberSuperOpen = 0.7;
    private double intaking = -1;
    private double spitout = 1;
    double currentX, currentY;
    private double WristDown = 0.65;
    private double WristUp = 0;
    private boolean errorBlockNotFound;
    private DcMotorEx extension;
    private int Retracted = 0;
    private int Home = 0;
    private int partialExtension = 180;
    //  private int halfExtension =
    // private int Maximum = 442;
    private int Maximum = 201;
    private int TopLevelLeft = 890;
    private int MiddleLevelLeft = 617;
    private int BottomLevelLeft = 375;
    private int TopLevelRight = -830;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private OpenCvInternalCamera phoneCam;
    private ShippingElementDetectorRedDepot detector = new ShippingElementDetectorRedDepot();
    private String position;
    private float leftVote = 0;
    private float centerVote = 0;
    private float rightVote = 0;
    private String FinalDecision;
    private int wait = 500;
    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        sensorColorRange_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        Claw.setPosition(grabberClose);
        sleep(2000);
        extension.setPower(-0.5);
        sleep(1000);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(0);
        extension.setPower(1);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        position = detector.position;
        phoneCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(16, -66, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        /*Trajectory warehouse = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(44, -62),Math.toRadians(180))
                .build();*/
        Trajectory hub = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(7,-48, Math.toRadians(55)))
                .addDisplacementMarker(0,() ->{
                    Arm.setTargetPosition(TopLevelLeft);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(6, ()->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })
                .addDisplacementMarker(10, ()->{
                    Claw.setPosition(grabberOpen);
                })
                //.splineToConstantHeading(new Vector2d(16,-66), Math.toRadians(180)) nerds
                //.splineTo(new Vector2d(8,-24), Math.toRadians(90)) ANDREW IS SO HOT
                //.splineToSplineHeading(new Pose2d(8,-24, Math.toRadians(-100)),Math.toRadians(90))
                .build();
        Trajectory warehouse1 = drive.trajectoryBuilder(hub.end())
                .splineToSplineHeading(new Pose2d(10,-61, Math.toRadians(0)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(52,-62), Math.toRadians(0))
                /*.addDisplacementMarker(0,() ->{
                    Claw.setPosition(grabberClose);
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(6,() ->{
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(22,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })*/
                .build();

        Trajectory hub1 = drive.trajectoryBuilder(warehouse1.end())
                .splineToConstantHeading(new Vector2d(30, -69), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12,-68), Math.toRadians(180))
                .splineTo(new Vector2d(11,-48), Math.toRadians(90))
                /*.addDisplacementMarker(0, ()->{
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })

                .addDisplacementMarker(50,() ->{
                    Arm.setTargetPosition(-TopLevelLeft);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(56, ()->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })
                .addDisplacementMarker(60, ()->{
                    Claw.setPosition(grabberOpen);
                })*/
                .build();

        Trajectory warehouse2 = drive.trajectoryBuilder(hub1.end())
                .splineToSplineHeading(new Pose2d(8,-65, Math.toRadians(0)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(52,-62), Math.toRadians(0))
                /*.addDisplacementMarker(0,() ->{
                    Claw.setPosition(grabberClose);
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(6,() ->{
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(22,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })*/
                .build();

        Trajectory hub2 = drive.trajectoryBuilder(warehouse2.end())
                .splineToConstantHeading(new Vector2d(30, -69), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12,-68), Math.toRadians(180))
                .splineTo(new Vector2d(11,-48), Math.toRadians(90))
                /*.addDisplacementMarker(0, ()->{
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })

                .addDisplacementMarker(50,() ->{
                    Arm.setTargetPosition(-TopLevelLeft);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(56, ()->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })
                .addDisplacementMarker(60, ()->{
                    Claw.setPosition(grabberOpen);
                })*/
                .build();

        Trajectory warehouse3 = drive.trajectoryBuilder(hub2.end())
                .splineToSplineHeading(new Pose2d(8,-65, Math.toRadians(0)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(52,-62), Math.toRadians(0))
                .addDisplacementMarker(0,() ->{
                    Claw.setPosition(grabberClose);
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(6,() ->{
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(22,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })
                .build();

        Trajectory hub3 = drive.trajectoryBuilder(warehouse3.end())
                .splineToConstantHeading(new Vector2d(30, -69), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12,-68), Math.toRadians(180))
                .addDisplacementMarker(0, ()->{
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .splineTo(new Vector2d(11,-48), Math.toRadians(90))
                .addDisplacementMarker(50,() ->{
                    Arm.setTargetPosition(-TopLevelLeft);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(56, ()->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })
                .addDisplacementMarker(60, ()->{
                    Claw.setPosition(grabberOpen);
                })
                .build();

        Trajectory warehouse4 = drive.trajectoryBuilder(hub3.end())
                .splineToSplineHeading(new Pose2d(8,-65, Math.toRadians(0)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(52,-62), Math.toRadians(0))
                .addDisplacementMarker(0,() ->{
                    Claw.setPosition(grabberClose);
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(6,() ->{
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(22,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })
                .build();

        Trajectory hub4 = drive.trajectoryBuilder(warehouse4.end())
                .splineToConstantHeading(new Vector2d(30, -69), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12,-68), Math.toRadians(180))
                .addDisplacementMarker(0, ()->{
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .splineTo(new Vector2d(11,-48), Math.toRadians(90))
                .addDisplacementMarker(50,() ->{
                    Arm.setTargetPosition(-TopLevelLeft);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(56, ()->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })
                .addDisplacementMarker(60, ()->{
                    Claw.setPosition(grabberOpen);
                })
                .build();
        Trajectory warehouse5 = drive.trajectoryBuilder(hub4.end())
                .splineToSplineHeading(new Pose2d(8,-65, Math.toRadians(0)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(52,-62), Math.toRadians(0))
                .addDisplacementMarker(0,() ->{
                    Claw.setPosition(grabberClose);
                    extension.setTargetPosition(0);
                    extension.setPower(1);
                })
                .addDisplacementMarker(6,() ->{
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                })
                .addDisplacementMarker(22,() ->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })
                .build();
        /*Trajectory goinwarehouse = drive.trajectoryBuilder(hub.end())
                .splineToLinearHeading(new Pose2d(16,-66, Math.toRadians(0)), Math.toRadians(0))

                //.splineToLinearHeading(new Pose2d(64, -63, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(5, ()->{
                 Arm.setTargetPosition(0);
                 Claw.setPosition(grabberClose);
                 Arm.setPower(1);
                })
                .addDisplacementMarker(8, ()->{
                    extension.setTargetPosition(partialExtension);
                    extension.setPower(1);
                })
                .addDisplacementMarker(25, ()->{
                    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Arm.setTargetPosition(0);
                    Arm.setPower(1);
                    Intake.setPower(intaking);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Claw.setPosition(grabberIntakeOpen);
                })
                .addDisplacementMarker(0, ()->{
                })
                .addDisplacementMarker(5, ()->{
                    extension.setTargetPosition(175);
                    extension.setPower(1);
                })
                .addDisplacementMarker(7, ()->{
                    Claw.setPosition(grabberOpen);
                })
                .build();
        Trajectory drive2 = drive.trajectoryBuilder(goinwarehouse.end())
                .splineToLinearHeading(new Pose2d(64,-63,Math.toRadians(0)),Math.toRadians(180))
                .build();
        Trajectory drive3 = drive.trajectoryBuilder(drive2.end())
                .splineToLinearHeading(new Pose2d(16,-66,Math.toRadians(0)),Math.toRadians(180))
                .build();*/
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
                telemetry.addData("LeftTotal", detector.Lefttotal);
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







        waitForStart();

//==================================================================================================================================


        if(isStopRequested()) return;


        //  drive.followTrajectory();
        //sleep(1000);
        drive.followTrajectory(hub);
        // sleep(150);
        drive.followTrajectory(warehouse1);
        //   sleep(150);
        drive.followTrajectory(hub1);
        //   sleep(150);
        drive.followTrajectory(warehouse2);
        //  sleep(150);
        drive.followTrajectory(hub2);
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
                Arm.setTargetPosition(BottomLevelLeft);
                Arm.setPower(1);
                break;
            case "Center":
                Arm.setTargetPosition(MiddleLevelLeft);
                Arm.setPower(1);
                break;
            case "Right":
                Arm.setTargetPosition(TopLevelLeft);
                Arm.setPower(1);
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
