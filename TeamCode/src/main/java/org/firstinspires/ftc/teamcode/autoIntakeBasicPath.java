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
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="autoIntake", group = "Knightrix")
public class autoIntakeBasicPath extends LinearOpMode{
    int test=0;
    boolean init=false;
    DcMotor leftDrive;
    DcMotor rightDrive;
    DistanceSensor distance;
    private DcMotor spinnerIntake = null;
    private DcMotor spinnerArm = null;
    TouchSensor BoxTouch;
    //    Servo servo;
    Servo servoFlip;
    CRServo contServo;
    int Drop_Rotation = 1200;
    int Drop_Range = 400;   //increase drop range
    int Pickup_Hover = 20;
    boolean dropping = false;
    double armTargetPos = 0;
    double armPos = 0;
    static double Right_TrigPosition;
    static double spinPower = 0;
    static double BoxPos = 1;
    boolean BoxInitialized = false;


    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        distance = hardwareMap.get(DistanceSensor.class, "distanceTest");
        BoxTouch = hardwareMap.get(TouchSensor.class, "Box_Touch");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        spinnerIntake = hardwareMap.get(DcMotor.class, "spinner_intake");
        spinnerArm = hardwareMap.get(DcMotor.class, "spinner_arm"); //spinner for big arm in intake
//        servo = hardwareMap.get(Servo.class, "servoTest");
        contServo =  hardwareMap.crservo.get("cont_Servo");
        servoFlip = hardwareMap.get(Servo.class, "flip_Intake");
        spinnerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Motor variables
        // Motors using encoders:
        spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //spinnerArm.setTargetPosition(200);
        // set motors to run to target encoder position and stop with brakes on.
        //spinnerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinnerArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // spinnerArm.setPower(1);

        // Motors without encoders
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        spinnerArm.setDirection(DcMotor.Direction.FORWARD);



        //leftDrive.setPower(0.2);
        //rightDrive.setPower(0.2);
        distance = hardwareMap.get(DistanceSensor.class, "distanceTest");
        //motor = hardwareMap.get(DcMotor.class, "Motor");

        // Loop while the Op Mode is running
        waitForStart();
// doesn't stop on turn
        while (opModeIsActive()) {
            double distance1 = distance.getDistance(DistanceUnit.CM);
            //Add data and format correctly
            telemetry.addData("status", "running");
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("test", ""+test);

            //Consistently update the data while the Op Mode is running
            telemetry.update();
            //moves until reached a certain distance
            if (distance1 < 60) {
                //run to position at the desiginated power
                leftDrive.setPower(-0.3);
                rightDrive.setPower(-0.3);
            } else {
                ///                       set code for turning
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                if (!init) {
                    InitBox();
                }

                // ***********************
                // **** MOVE THE ARM *****
                // ***********************
                // Select PICKUP or DROP POSITION for the Arm


                if (gamepad2.a) {
                    dropping = true;
                    armTargetPos = Drop_Rotation;
                } else if (gamepad2.b) {
                    dropping = false;
                    armTargetPos = Pickup_Hover;
                    if (!BoxTouch.isPressed()){
                        spinnerArm.setPower(-0.5);
                    } else {
                        spinnerArm.setPower(0);
                    }

                }
                // Adjust Drop Position with Gamepad for 1, 2 & 3 levels
                if (dropping) {
                    Right_TrigPosition = armTargetPos + (int)(Drop_Range * gamepad2.left_trigger);
                    test++;
                } else {
                    Right_TrigPosition = armTargetPos + 0;
                    telemetry.addData("down", "");
                    telemetry.update();
                }

                // Calculate the power for the Arm and Move
                CalcArmPower((int)Right_TrigPosition);
                spinnerArm.setPower(spinPower);
            }
            //spin(5000);
        }
    }

    private void InitBox(){
        init = true;
        spinnerArm.setPower(-0.6);              // Moves Arm UP
        armPos = spinnerArm.getCurrentPosition();

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

    private void CalcBoxPosition(double armPos) {
        // Define position variables
        double PickUpStart = 0;
        double PickUpPos = 0.17;
        double DropPos= .65;
        double Trigger_Adjust = gamepad2.right_trigger + gamepad1.right_trigger; // Values 0 to 1

        if (!dropping) { //If we are picking up blocks
            BoxPos = PickUpStart + (Trigger_Adjust * (PickUpPos-PickUpStart));   //safe position + Driver Adjustment
        } else {         // Dropping blocks;
            // Once reached Delivery, Use joystick
            if ((int)Math.abs(armPos-Drop_Rotation)<200) { // We are close to drop zone
                BoxPos = DropPos - Trigger_Adjust * (1-DropPos); // To ensure value is not bigger than 1
            } else {                                       // We are travelling
                // Between PickUp and Delivery, calculate balance
                BoxPos = (armPos / Drop_Rotation * (DropPos-PickUpStart) ) + PickUpStart;
            }
        }
        BoxPos = Range.clip(BoxPos, 0, 1);
    }


    private void CalcArmPower(int right_trigPosition) {
        // 1.Get current position
        // 2.Compare with target position
        // 3.Determine Side (Drop / PickUP)
        // 4.Determine Power need (High / Low)
        // 5.High Power => Power 100%
        // 6.Low Power => Power 30%
        // 7. Mid Power => Power 50%

        // Variables
        double OvershootPower = 0;
        double FullPower = .6; //change to 1 for robot
        int MoveThreshold = 5;
        // 1. Get current position
        armPos = spinnerArm.getCurrentPosition();

        if (armPos < 600) { // Pickup Zone
            if(armPos > Right_TrigPosition) {   // Above Target
                spinPower = OvershootPower;    //was a negative
            } else {
                // Add continuous down when close to Pickup_Hover position
                // until the Touch is triggered
                if (dropping) {
                    spinPower = -FullPower;  // Go UP
                } else {
                    spinPower = OvershootPower; // Go Down
                }
            }
        } else if (armPos > 1000) { // Drop Zone
            if(armPos < Right_TrigPosition) {   // Above Target
                spinPower = -OvershootPower;
            } else {
                spinPower = FullPower;
            }
        } else { // Middlezone
            if(armPos > Right_TrigPosition) {
                spinPower = .4;
            } else {
                spinPower = -.4;
            }
        }

        // if within threshold, no more power
        if (Math.abs(Right_TrigPosition - armPos) < MoveThreshold) { //Inside Threshold don't move
            if(dropping) {
                spinPower = .1;
            }
        }

        // going Picking up AND trigger then
        // stop motors
        // set encoder to zero
        if ((! dropping) && BoxTouch.isPressed()) {
            spinPower = 0; //Turn OFF Motor
            spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //This will also reset armPos
            spinnerArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

}
