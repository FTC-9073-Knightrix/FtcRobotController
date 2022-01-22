package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class AutoInitilization extends LinearOpMode{

    private DcMotor spinnerArm = null;
    double armPos = 0;
    Servo servoFlip;
    TouchSensor BoxTouch;
    boolean BoxInitialized = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        spinnerArm = hardwareMap.get(DcMotor.class, "spinner_arm"); //spinner for big arm in intake
        servoFlip = hardwareMap.get(Servo.class, "flip_Intake");
        BoxTouch = hardwareMap.get(TouchSensor.class, "Box_Touch");


        spinnerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinnerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinnerArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerArm.setDirection(DcMotor.Direction.FORWARD);



        //leftDrive.setPower(0.2);
        //rightDrive.setPower(0.2);
        //distance = hardwareMap.get(DistanceSensor.class, "distanceTest");
        //motor = hardwareMap.get(DcMotor.class, "Motor");

        // Loop while the Op Mode is running
        waitForStart();

        InitBox();
    }

    private void InitBox(){
        spinnerArm.setPower(-0.6);              // Moves Arm UP  ***************changed from negative to positive
        armPos = spinnerArm.getCurrentPosition();
        //add spinner intake if needed

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

}

