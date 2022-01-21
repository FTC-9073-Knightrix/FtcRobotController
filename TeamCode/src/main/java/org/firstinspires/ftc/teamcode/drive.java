package org.firstinspires.ftc.teamcode;


import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.source.tree.WhileLoopTree;


@TeleOp(name="Drive", group="Knightrix")
public class drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor spinnerIntake = null;
    private DcMotor spinnerArm = null;
    TouchSensor BoxTouch;
//    Servo servo;
    Servo servoFlip;
    CRServo contServo;

    // Variables
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

    //right trigger move one motor more depending on 1 or -1 values (Range.clip())
    //encoder used to move intake
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        // Adding touch sensor as hardware program.
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


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Define Variables
        // Setup a variable for each drive wheel to save power level for telemetry
        double LeftPower;
        double RightPower;
        double max;

        double rightPower;
        double servoposition;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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
            } else {
                Right_TrigPosition = armTargetPos + 0;
            }

            // Calculate the power for the Arm and Move
            CalcArmPower((int)Right_TrigPosition);
            spinnerArm.setPower(spinPower);

            /*  //Keep going down until touch sensor is triggered
            if (BoxTouch.isPressed()) {
                armPos = 0;
                spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                if (spinnerArm.getCurrentPosition() <= Pickup_Hover) {
                    armPos = armPos - 5;
                }
            }
            */



            // *******************
            // ** MOVE THE BOX ***
            // *******************
            // if pickup => safe postion + Joystick to go down
            CalcBoxPosition((double)armPos);

            if (!BoxInitialized) {
                InitBox();
            } else {
                servoFlip.setPosition(BoxPos);
            }





            // *************************
            // **** DRIVE THE ROBOT ****
            // *************************
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            LeftPower  = drive - turn;
            RightPower = drive + turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(LeftPower), Math.abs(RightPower));
            if (max > 1.0)
            {
                LeftPower /= max;
                RightPower /= max;
            }

            // Output the safe vales to the motor drives.
            leftDrive.setPower(LeftPower);
            rightDrive.setPower(RightPower);




            // ************************
            // *** MOVE THE INTAKE ****
            // ************************
            int intakePower = 0;
            if (gamepad2.dpad_up || gamepad1.dpad_up) { intakePower = 1;};
            if (gamepad2.dpad_down || gamepad1.dpad_down) { intakePower = -1;};
            // Set hex motor to game pad speed
            // Set hex motor to game pad speed
            spinnerIntake.setPower((double) intakePower);




            // ****************************
            // *** MOVE THE DUCK SERVO ****
            // ****************************
            // Set servo power to highest
            if (gamepad1.right_bumper) {
                contServo.setPower(1.0);
            } else if (gamepad1.left_bumper){
                contServo.setPower(-1.0);
            } else {
                contServo.setPower(0);
            }



            // TO REVIEW
            //servoposition = Range.clip(gamepad1.left_trigger, 0, 1);



            // *****************
            // *** TELEMETRY ***
            // *****************
            telemetry.addData("Move to:", Right_TrigPosition);
            telemetry.addData("Initial: ", BoxInitialized);
            telemetry.addData("Curr Pos:",armPos);
            telemetry.addData("Power:",spinPower);
            telemetry.addData("Box Target:",BoxPos);
            telemetry.addData("Box:",servoFlip.getPosition());
            telemetry.addData("is at target", !spinnerArm.isBusy());

            telemetry.addData("Dropping?:", dropping);
            telemetry.update();

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", drivePower, turnPower);
            //telemetry.update();

            // If touch sensor hits the ground, set servo power to 0
            //if (touch.isPressed()) {
            //null;
            //}

        }
    }

    private void InitBox(){
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
