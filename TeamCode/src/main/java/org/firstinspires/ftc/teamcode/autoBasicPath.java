package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name = "red_Competition", group = "Knightrix")
public class autoBasicPath extends LinearOpMode{

    DcMotor leftDrive;
    DcMotor rightDrive;
    BNO055IMU imu;
    Orientation angles;
    DistanceSensor distance;
    private DcMotor spinnerArm = null;
    double armPos = 0;
    Servo servoFlip;
    TouchSensor BoxTouch;
    boolean BoxInitialized = false;
    private DcMotor spinnerIntake = null;



    boolean status = false;
    boolean done = false;
    boolean fback = false;
    CRServo crServo;
    private ElapsedTime runtime = new ElapsedTime();


    public double getDistance(){
        return distance.getDistance(DistanceUnit.CM);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        distance = hardwareMap.get(DistanceSensor.class, "distanceTest");
        crServo = hardwareMap.get(CRServo.class, "cont_Servo");
        spinnerArm = hardwareMap.get(DcMotor.class, "spinner_arm"); //spinner for big arm in intake
        servoFlip = hardwareMap.get(Servo.class, "flip_Intake");
        BoxTouch = hardwareMap.get(TouchSensor.class, "Box_Touch");
        spinnerIntake = hardwareMap.get(DcMotor.class, "spinner_intake");



        spinnerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinnerArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerArm.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //leftDrive.setPower(0.2);
        //rightDrive.setPower(0.2);
        //distance = hardwareMap.get(DistanceSensor.class, "distanceTest");
        //motor = hardwareMap.get(DcMotor.class, "Motor");

        // Loop while the Op Mode is running
        waitForStart();
        runtime.reset();

        InitBox();
        //Store distance sensor
        double distance1 = distance.getDistance(DistanceUnit.CM);
        distance_movement(38,0.9);  //makes forward from wall
        imuGyroTurn(90, .9);  //turns towards right
        distance_movement(19, 0.6);  //moves backwards torwards wall
        imuGyroTurn(0, -.9);  //turns towards right
        //distance_movement(32, 0.9);  //moves backwards torwards wall
        crServo.setPower(1);
        leftDrive.setPower(.2);
        rightDrive.setPower(.2);

        sleep(6000);

        //Go back to base
        crServo.setPower(0);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        distance_movement(57.8,0.9);  //makes forward from wall
        InitBox();



        //spin(5000);

    }

    private void InitBox(){
        spinnerArm.setPower(-0.6);              // Moves Arm UP  ***************changed from negative to positive
        armPos = spinnerArm.getCurrentPosition();
        //spinnerIntake.setPower(1);  //  spins intake the entire time
        while (armPos < 300){
            armPos = spinnerArm.getCurrentPosition();
            telemetry.addData("ArmPos: ", armPos);
            telemetry.update();
        }

        servoFlip.setPosition(0);              // Moves Servo to safe position
        spinnerArm.setPower(-0.2);              // To hold Arm in place
        sleep(2000);                 // Wait for BOx to go to safe position

        while (!BoxTouch.isPressed() ){  //add an or statement for #seconds passed
            spinnerArm.setPower(0);              // Moves Arm DOWN

        }

        spinnerArm.setPower(0);              //Turn OFF Motor
        spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //This will also reset armPos
        BoxInitialized = true;

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


    private void distance_movement(double distance, double speed) {

        //Store distance sensor
        double currDist = getDistance();

        while (opModeIsActive() && Math.abs(currDist - distance)  > 2 ) {
            currDist = getDistance();
            telemetry.addData("Current distance", ""+currDist);
            telemetry.addData("Desired Distance", ""+distance);
            telemetry.addData("Difference", Math.abs(currDist - distance));
            telemetry.update();

            if ((currDist - distance) > 0 ) {
                leftDrive.setPower(speed);
                rightDrive.setPower(speed);
            } else {
                leftDrive.setPower(-speed);
                rightDrive.setPower(-speed);
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }


    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void imuGyroTurn(double turnDegrees, double speed){

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double Curr_Heading = angles.firstAngle;
        double Target_Heading = (Curr_Heading*0) - turnDegrees;


        while ((opModeIsActive()) && (Math.abs(Target_Heading-Curr_Heading)>5)){   //5 is our threshold for angle


            //where am I? => Variable
            //loop to check I'm off that target
            leftDrive.setPower(-speed);
            rightDrive.setPower(speed);

            Curr_Heading = angles.firstAngle;

            if (Curr_Heading > 180) {
                Curr_Heading -= 360;
            } else if (Curr_Heading < -180) {
                Curr_Heading += 360;
            }
            //end loop

            telemetry.update();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Curr_Heading: ", Curr_Heading);
            telemetry.addData("Target_Heading: ",Target_Heading);
            telemetry.addData("TurnDegrees: ",turnDegrees);
            telemetry.addData("Abs difference: ",Math.abs(Target_Heading-Curr_Heading));
            telemetry.addData("Right Power", rightDrive.getPower());
            telemetry.addData("Left Power", leftDrive.getPower());

        }
        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);


    }
}
