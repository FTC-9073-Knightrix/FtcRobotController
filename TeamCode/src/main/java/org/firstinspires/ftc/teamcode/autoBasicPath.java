package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "autoBasicPath", group = "Knightrix")
public class autoBasicPath extends LinearOpMode{

    DcMotor leftDrive;
    DcMotor rightDrive;
    DistanceSensor distance;


    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    double distance1;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException{

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        distance = hardwareMap.get(DistanceSensor.class, "distanceTest");

        //leftDrive.setPower(0.2);
        //rightDrive.setPower(0.2);
        distance = hardwareMap.get(DistanceSensor.class, "distanceTest");
        //motor = hardwareMap.get(DcMotor.class, "Motor");

        // Loop while the Op Mode is running
        waitForStart();
// doesn't stop on turn
        while (opModeIsActive()) {
            double distance1 = distance.getDistance(DistanceUnit.CM);
            boolean turn = false;
            //Add data and format correctly
            telemetry.addData("status", "running");
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            //Consistently update the data while the Op Mode is running
            telemetry.update();
            //moves until reached a certain distance
            if (distance1 < 40) {
                //run to position at the desiginated power
                leftDrive.setPower(-0.3);
                rightDrive.setPower(-0.3);
                telemetry.addData("turning", "false");
                telemetry.update();
            } else {
                ///                       set code for turning
                //leftDrive.setPower(0);
                //rightDrive.setPower(0);
                leftDrive.setPower(0.5);
                rightDrive.setPower(-0.5);
                sleep(1000);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                telemetry.addData("turning:", "true");
                telemetry.update();

            }
            //spin(5000);
        }
    }



    public void spin(long time){
        while (opModeIsActive() && (getRuntime() < time)){
            //leftDrive.setPower(0.2);
            //rightDrive.setPower(0.2);

            telemetry.addData("Status:", "Running");
            //telemetry.addData("Power", DcMotor.getPower());
            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.addData("distance: ", "" + distance1);
            telemetry.update();
        }
    }

    private void drive(double power, double leftInches, double rightInches) {
        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            double distance1 = distance.getDistance(DistanceUnit.CM);
            //Add data and format correctly
            telemetry.addData("status", "running");
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            //Consistently update the data while the Op Mode is running
            telemetry.update();
            // Create target positions
            rightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * DRIVE_COUNTS_PER_IN);
            leftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * DRIVE_COUNTS_PER_IN);

            // set target position
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            //switch to run to position mode
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            }

            // set motor power back to 0
            //leftDrive.setPower(0);
            //rightDrive.setPower(0);
        }
    }
}
