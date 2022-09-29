package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ShippingElementDetectorBlueCarousel extends OpenCvPipeline {
    Mat workingMatrix = new Mat();
    Mat workingMatrix2 = new Mat();
    public String position = "Left";
    public Double Lefttotal;
    public Double Righttotal;
    public Double Centertotal;
    public ShippingElementDetectorBlueCarousel() {

    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix2, Imgproc.COLOR_RGB2YCrCb);
        //BGR2HSV
/*
        Mat matLeft = workingMatrix2.submat(500, 560, 210, 270);
        Mat matCenter = workingMatrix2.submat(260, 320, 210, 270);
        Mat matRight = workingMatrix2.submat(10, 70, 210, 270);
*/
        Mat matLeft2 = workingMatrix2.submat(560, 620, 210, 270);
        Mat matCenter2 = workingMatrix2.submat(320, 380, 210, 270);
        Mat matRight2 = workingMatrix2.submat(70, 130, 210, 270);

        Imgproc.rectangle(workingMatrix2, new Rect(240, 530, 60, 60), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix2, new Rect(240, 290, 60, 60), new Scalar(255, 255, 255));
        Imgproc.rectangle(workingMatrix2, new Rect(240, 40, 60, 60), new Scalar(0, 0, 255));
// COLUMN = X  ROW = Y
        double leftTotal = Core.sumElems(matLeft2).val[2];
        double rightTotal = Core.sumElems(matRight2).val[2];
        double centerTotal = Core.sumElems(matCenter2).val[2];
        Lefttotal = leftTotal;
        Righttotal = rightTotal;
        Centertotal = centerTotal;
        if ((leftTotal < rightTotal) && (leftTotal < centerTotal)) {
            position = "Left";
        }   else if ((rightTotal < centerTotal) && (rightTotal < leftTotal)) {
            position = "Right";
        }   else if ((centerTotal < rightTotal) && (centerTotal < leftTotal)){
            position = "Center";
        }


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
                    //right is shipping element
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



 /*       if (leftTotal > rightTotal) {
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
*/        return workingMatrix2;
    }
}

