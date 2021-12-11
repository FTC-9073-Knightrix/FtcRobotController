package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Arm_Direct_Drive", group="Testing")
public class ArmDrive extends LinearOpMode {
    DcMotor spinnerIntake = null;
    DcMotor spinnerArm = null;
    TouchSensor BoxTouch;
    Servo servoFlip;

    // Variables
    double armPos = 0;
    static double spinPower = 0;
    static double BoxPos = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone)
        BoxTouch = hardwareMap.get(TouchSensor.class, "Box_Touch");
        spinnerIntake = hardwareMap.get(DcMotor.class, "spinner_intake");
        spinnerArm = hardwareMap.get(DcMotor.class, "spinner_arm"); //spinner for big arm in intake
        servoFlip = hardwareMap.get(Servo.class, "flip_Intake");

        // Set Motor variables
        // Motors without encoders
        spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinnerArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerArm.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Define Variables
        // Setup a variable for each drive wheel to save power level for telemetry
        double max;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // ***********************
            // **** MOVE THE ARM *****
            // ***********************
            // Uses values between -1 & +1
            spinPower = gamepad1.right_stick_y;

            // Reset Encoder when 0
            if (BoxTouch.isPressed()) {
                spinPower = 0; //Turn OFF Motor
                spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //This will also reset armPos
            }

            // Moves the Arm
            spinnerArm.setPower(spinPower);

            // Get the location of the arm
            armPos = spinnerArm.getCurrentPosition();


            // *******************
            // ** MOVE THE BOX ***
            // *******************
            // Uses values between 0 & 1
            BoxPos = (gamepad1.left_stick_y +1)/2;
            servoFlip.setPosition(BoxPos);


            // *************************
            // **** DRIVE THE ROBOT ****
            // *************************


            // ************************
            // *** MOVE THE INTAKE ****
            // ************************
            double intakePower = gamepad2.left_stick_y;
            spinnerIntake.setPower(intakePower);


            // ****************************
            // *** MOVE THE DUCK SERVO ****
            // ****************************





            // *****************
            // *** TELEMETRY ***
            // *****************
            telemetry.addData("Curr Pos:",armPos);
            telemetry.addData("Power:",spinPower);
            telemetry.addData("Box:",BoxPos);
            telemetry.addData("is at target", !spinnerArm.isBusy());

            telemetry.update();


        }
    }



        // going Picking up AND trigger then
        // stop motors
        // set encoder to zero
    }







