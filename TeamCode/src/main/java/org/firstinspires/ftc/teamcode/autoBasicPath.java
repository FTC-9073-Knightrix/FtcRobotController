package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@Autonomous(name = "autoBasicPath", group = "Knightrix")
public class autoBasicPath{
    public DcMotor LeftDrive = null;
    /*
    public DcMotor RightDrive = null;
    public DcMotor LeftDrive = null ;
    public DcMotor Arm;
    public DcMotor spinnerArm;

    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Drive function with 3 parameters
    private void drive(double power, double leftInches, double rightInches) {
        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            // Create target positions
            rightTarget = RightDrive.getCurrentPosition() + (int) (rightInches * DRIVE_COUNTS_PER_IN);
            leftTarget = LeftDrive.getCurrentPosition() + (int) (leftInches * DRIVE_COUNTS_PER_IN);

            // set target position
            LeftDrive.setTargetPosition(leftTarget);
            RightDrive.setTargetPosition(rightTarget);

            //switch to run to position mode
            LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the desiginated power
            LeftDrive.setPower(power);
            RightDrive.setPower(power);

            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (LeftDrive.isBusy() || RightDrive.isBusy())) {
            }

            // set motor power back to 0
            LeftDrive.setPower(0);
            RightDrive.setPower(0);
        }
    }


    @Override
    public void runOpMode() {

        spinnerArm.setPower(-0.2);              // To hold Arm in place

        RightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        LeftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        spinnerArm = hardwareMap.get(DcMotor.class, "spinner_arm");


        LeftDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {

            //segment 1
            drive(0.7, 30, 15);

            runtime.reset(); // reset elapsed time timer

            //segment 2 - lift arm, drive to shipping hub, outtake freight
            while (opModeIsActive() && runtime.seconds() <= 7) {

                //lift arm and hold
                Arm.setTargetPosition(120);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.3);

                //drive forward for 1 second
                while (runtime.seconds() > 2 && runtime.seconds() <= 3) {
                    drive(0.4, 4, 4);
                }

                //run intake
                while (runtime.seconds() > 4 && runtime.seconds() <= 7) {
                    spinnerArm.setPower(-0.6);
                }

                // turn off arm and intake
                Arm.setPower(0);
                spinnerArm.setPower(0);

                //segment 3 - reverse to get better angle
                drive(0.7, -15, -30);

                //segment 4 - drive into warehouse
                drive(1, 90, 90);
            }
        }
    }*/
}