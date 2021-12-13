package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "testAuto", group = "Knightrix")
public class autoProgram extends LinearOpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;
    public void moveForward(double time) {
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        sleep((long)time);
    }
    public void turnRight(double time){
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        sleep((long)time);
    }
    public void turnLeft(double time){
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        sleep((long)time);
    }
    public void setOff(){
        //setting power to zero (stop)
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }


    @Override
    public void runOpMode() throws InterruptedException{
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        waitForStart();
        moveForward(1000);
        turnLeft(1000);
        moveForward(1000);
        wait(100);


        }
    }

