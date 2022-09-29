package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import org.opencv.core.*;
import org.opencv.core.Core.*;
//import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
@Disabled
public class ShippingElementDetector extends OpenCvPipeline {
    private Mat resizeImageOutput = new Mat();
    private Mat cvThresholdOutput = new Mat();
    private Mat cvExtractchannelOutput = new Mat();

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
private Mat workingMatrix = new Mat();
public String position = "Left";
    public ShippingElementDetector() {

    }
    public void process(Mat source0) {
        // Step Resize_Image0:
        Mat resizeImageInput = source0;
        double resizeImageWidth = 640;
        double resizeImageHeight = 480;
        int resizeImageInterpolation = Imgproc.INTER_CUBIC;
        resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);

        // Step CV_Threshold0:
        Mat cvThresholdSrc = resizeImageOutput;
        double cvThresholdThresh = 150.0;
        double cvThresholdMaxval = 200.0;
        int cvThresholdType = Imgproc.THRESH_BINARY_INV;
        cvThreshold(cvThresholdSrc, cvThresholdThresh, cvThresholdMaxval, cvThresholdType, cvThresholdOutput);

        // Step CV_extractChannel0:
        Mat cvExtractchannelSrc = cvThresholdOutput;
        double cvExtractchannelChannel = 0;
        cvExtractchannel(cvExtractchannelSrc, cvExtractchannelChannel, cvExtractchannelOutput);

    }
    public Mat resizeImageOutput() {
        return resizeImageOutput;
    }
    public Mat cvThresholdOutput() {
        return cvThresholdOutput;
    }
    public Mat cvExtractchannelOutput() {
        return cvExtractchannelOutput;
    }
    private void resizeImage(Mat input, double width, double height,
                             int interpolation, Mat output) {
        Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }
    private void cvThreshold(Mat src, double threshold, double maxVal, int type,
                             Mat dst) {
        Imgproc.threshold(src, dst, threshold, maxVal, type);
    }
    private void cvExtractchannel(Mat src, double channel, Mat dst) {
        Core.extractChannel(src, dst, (int)channel);
    }
    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }


        //Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);
/*
        Mat matLeft = workingMatrix.submat(210,270,60,120);
        Mat matCenter = workingMatrix.submat(210,270,290,350);
        Mat matRight = workingMatrix.submat(210,270,520,580);
*/
        Mat matLeft = workingMatrix.submat(120, 180, 210, 270);
        Mat matCenter = workingMatrix.submat(290, 350, 210, 270);
        Mat matRight = workingMatrix.submat(460, 520, 210, 270);

        Imgproc.rectangle(workingMatrix, new Rect(240, 150, 60, 60), new Scalar(255, 0, 255));
        Imgproc.rectangle(workingMatrix, new Rect(240, 320, 60, 60), new Scalar(255, 0, 255));
        Imgproc.rectangle(workingMatrix, new Rect(240, 490, 60, 60), new Scalar(255, 0, 255));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];

        /*if ((leftTotal > rightTotal) && (leftTotal > centerTotal)) {
            position = "Left";
        }   else if ((rightTotal > centerTotal) && (rightTotal > leftTotal)) {
            position = "Right";
        }   else {
            position = "center";
        }*/


        /*if (leftTotal > rightTotal) {
            if (leftTotal > centerTotal) {
                position = "Left";
                //left is shipping element
            } else {
                //right is shipping element
                position = "Right";
            }
        } else {
                if (centerTotal > rightTotal) {
                    //center is shipping element
                    position = "Center";
                } else {
                    //right is skystone
                    position = "Right";
                }
            }*/
/*
        if (leftTotal > rightTotal) {
            if (leftTotal > centerTotal) {
                position = "Left";
                //left is shipping element
            } else {
                //right is shipping element
                position = "Center";
            }
        } else {
            if (centerTotal > rightTotal) {
                //center is shipping element
                position = "Center";
            } else {
                //right is skystone
                position = "Right";
            }
        }
*/
        if (leftTotal > rightTotal) {
            if (leftTotal > centerTotal) {
                position = "Left";
                //left is shipping element
            } else {
                //right is shipping element
                position = "Center";
            }
        } else {
            if (centerTotal > rightTotal) {
                //center is shipping element
                position = "Center";
            } else {
                //right is skystone
                position = "Right";
            }
        }
        return workingMatrix;
    }
}
