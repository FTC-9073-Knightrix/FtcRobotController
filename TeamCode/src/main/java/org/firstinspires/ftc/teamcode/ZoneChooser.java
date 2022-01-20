package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "zoneAuto", group = "Knightrix")
public class ZoneChooser extends OpMode {
    DcMotor m1 = null;


    OpenCvCamera phoneCam = null;

    int rect1x = 0;
    int rect1y = 0;

    @Override
    public void init() {
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m1.setDirection(DcMotor.Direction.FORWARD);
        m1.setPower(1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

    }

    @Override
    public void init_loop() {
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(new RingDetectingPipline());
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void loop() {

    }

    class RingDetectingPipline extends OpenCvPipeline {
        Mat outPut = new Mat();
        Mat YCbCr = new Mat();
        Mat lowerCrop = new Mat();  // cropping: get specific section of color on image

        @Override
        public Mat processFrame(Mat input) {
            // visual code
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            //input.copyTo(outPut);

            // rectangles for detection
            //Rect rect = new Rect((int) Math.round(YCbCr.rows() * 0.18), rect1y, 119, 69); // top left of rectangle
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);
            return input;

            // scalar: holds 3 different values
            //Scalar rectColor = new Scalar(0, 0, 255); // rgb value for rectangle in scalar

            //Imgproc.rectangle(outPut, rect, rectColor, 2);

            // cropping the image for pic height
            //lowerCrop = YCbCr.submat(rect);  // create sub region of image (cropped rect region)

            // extracting channels: channels are an array (list of numbers 0, 1, 2...)
            // channels: YCbCr (Cb = channel 2)
            // taking the orange color out and placing on mat
            //Core.extractChannel(lowerCrop, lowerCrop, 2); // use lowerCrop and put extracting data on lowerCrop

            // take average (stored in Scalar) and get first value
            //Scalar lowerAverageOrange = Core.mean(lowerCrop); // raw average data and put into Scalar var
            // first color: Cr color
            //double finalLowerAverage = lowerAverageOrange.val[0];

            /*if (finalLowerAverage > 15 && finalLowerAverage < 130) {
                telemetry.addData("color", "orange");
                telemetry.update();
            }*/

            // what is showing on viewport
            //return outPut;
        }
    }
}/*

    /*Mat mat = new Mat();
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
    }*/

