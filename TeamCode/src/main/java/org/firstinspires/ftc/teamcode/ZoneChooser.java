package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ZoneChooser extends OpenCvPipeline {
    Mat mat = new Mat();
    Rect upperROI = new Rect(new Point(240, 120), new Point(304, 145));
    Rect lowerROI = new Rect (new Point(240,145), new Point(304, 170));
    Mat upperMat;
    Mat lowerMat;
    @Override
    public Mat processFrame(Mat input) {
        // thresholding
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowerBound = new Scalar(15.0 / 2, 100, 100);
        Scalar upperBound = new Scalar(45.0/2, 255, 255);
        Core.inRange(mat, lowerBound, upperBound, mat);

        // divide
        upperMat = mat.submat(upperROI);
        lowerMat = mat.submat(lowerROI);

        // average
        // see how much orange in each region
        // percentage of orange pixels
        double upperValue = Math.round(Core.mean(upperMat).val[2] / 255); // (2) = v in hsv
        double lowerValue = Math.round(Core.mean(lowerMat).val[2] / 255); // (2) = v in hsv
        upperMat.release();
        lowerMat.release();
        mat.release();

        // compare


        return null;
    }
}
