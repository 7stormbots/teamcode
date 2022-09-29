package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
@Disabled
@Autonomous (name="OpenCVtest", group = "")
public class OpenCVtest extends LinearOpMode {
    private OpenCvInternalCamera phoneCam;
    private ShippingElementDetector detector = new ShippingElementDetector();
    private String position;
    private float leftVote = 0;
    private float centerVote = 0;
    private float rightVote = 0;
    private String FinalDecision;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        phoneCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        position = detector.position;
      //  boolean Left = false;
        //boolean Right = false;
        //boolean Center = false  ;
        //double leftVote = 0;
        //double rightVote = 0;
        //double centerVote = 0;
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

    }
}
