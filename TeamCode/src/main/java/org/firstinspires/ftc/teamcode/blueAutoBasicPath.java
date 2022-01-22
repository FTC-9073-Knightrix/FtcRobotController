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

@Autonomous(name = "blueAautoBasicPath", group = "Knightrix")
public class blueAutoBasicPath extends LinearOpMode{

    DcMotor leftDrive;
    DcMotor rightDrive;
    DistanceSensor distance;
    boolean status = false;
    boolean done = false;
    boolean fback = false;
    CRServo crServo;
    private ElapsedTime runtime = new ElapsedTime();



    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    double distance1;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        distance = hardwareMap.get(DistanceSensor.class, "distanceTest");
        crServo = hardwareMap.get(CRServo.class, "cont_Servo");


        //leftDrive.setPower(0.2);
        //rightDrive.setPower(0.2);
        //distance = hardwareMap.get(DistanceSensor.class, "distanceTest");
        //motor = hardwareMap.get(DcMotor.class, "Motor");

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive() && !done){
            // doesn't stop on turn
            double distance1 = distance.getDistance(DistanceUnit.CM);
            //boolean turn = false;
            //Add data and format correctly
            telemetry.addData("status", "running");
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            //Consistently update the data while the Op Mode is running
            telemetry.update();
            //moves until reached a certain distance
            if (distance1 < 40 && !status) {
                //run to position at the desiginated power
                telemetry.addData("running", "forward");
                telemetry.update();
                leftDrive.setPower(0.6);
                rightDrive.setPower(0.6);

            } else if (distance1>40 && !status){
                leftDrive.setPower(-0.75); //starts turning
                rightDrive.setPower(0.75);
                Thread.sleep(1000);
                status = true;
            }
            //backwards towards carosel and turns
            if(distance1>50 && status){
                distance1 = distance.getDistance(DistanceUnit.CM);
                telemetry.addData("distance after turn: ", distance.getDistance(DistanceUnit.CM));
                telemetry.update();
                leftDrive.setPower(-0.6);
                rightDrive.setPower(-0.6);
                //Thread.sleep(700);
            } else if (distance1<40 && status && !fback) {
                leftDrive.setPower(0.75); //starts turning
                rightDrive.setPower(-0.75);
                Thread.sleep(1000);
                fback = true;
            }

            if (distance1>50 && fback) {
                telemetry.addData("dist:", ""+distance1);
                leftDrive.setPower(-0.6);
                rightDrive.setPower(-0.6);
            } else{
                spin(5000);
                done = true;
            }

            //leftDrive.setPower(0.5); //starts going straight after turn so it goes over barricade and goes to warehouse
            //rightDrive.setPower(0.5);
            //Thread.sleep(2000);
        }
    }

    public void spin(long time){
        while (opModeIsActive()){
            if (getRuntime() < time){
                crServo.setPower(1);
                telemetry.addData("Status:", "Running");
                telemetry.addData("Power", crServo.getPower());
                telemetry.addData("Status", "Run Time: " + getRuntime());
                telemetry.update();
            }
            else {
                crServo.setPower(0);
            }
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
