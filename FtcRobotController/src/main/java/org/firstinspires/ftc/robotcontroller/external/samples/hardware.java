package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class hardware extends BasicOpMode_Linear{

    //Declaring Motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftRearDrive;
    public DcMotor rightRearDrive;

    //Declaring Servos
    public Servo servo1;
    public Servo servo2;
    public Servo servo3;
    public Servo servo4;

    //Create the variable that will keep track of the left joystick's x value
    public float leftstick_x = 0;
    public float g1_leftstick_x = 0;
    public float g2_leftstick_x = 0;
    //Create the variable that will keep track of the left joystick's y value
    public float leftstick_y = 0;
    public float g1_leftstick_y = 0;
    public float g2_leftstick_y = 0;

    public float g1_rightstick_x = 0;
    public float g2_rightstick_x = 0;

    //Create the variable that tracks the GamePad buttons
    public boolean g2_dpad_down  = false;
    public boolean g2_dpad_up    = false;
    public boolean g2_dpad_right = false;
    public boolean g2_dpad_left  = false;

    // Game pad Bumpers
    public boolean g1_left_bumper = false;
    public boolean g1_right_bumper = false;
    public boolean g2_right_bumper = false;
    public boolean g2_left_bumper  = false;

    //Gamepad triggers
    public float g2_right_trigger = 0;
    public float g2_left_trigger = 0;

    //Gamepad buttons
    public boolean g1_a = false;
    public boolean g1_b = false;
    public boolean g2_a = false;
    public boolean g2_b = false;
    public boolean g2_y = false;
    public boolean g1_dpad_up = false;
    public boolean g1_dpad_down = false;

    public void getController() {
        //Gamepad
        g1_leftstick_x = gamepad1.left_stick_x;
        g2_leftstick_x = gamepad2.left_stick_x;

        g1_leftstick_y = gamepad1.left_stick_y;
        g2_leftstick_y = gamepad2.left_stick_y;

        g1_rightstick_x = gamepad1.right_stick_x;
        g2_rightstick_x = gamepad2.right_stick_x;


        //Gamepad buttons
        g1_dpad_down = gamepad1.dpad_down;
        g1_dpad_up = gamepad1.dpad_up;
        g2_dpad_down = gamepad2.dpad_down;
        g2_dpad_up   = gamepad2.dpad_up;
        g2_dpad_right= gamepad2.dpad_right;
        g2_dpad_left = gamepad2.dpad_left;
        g1_a = gamepad1.a;
        g1_b = gamepad1.b;
        g2_a = gamepad2.a;
        g2_b = gamepad2.b;
        g2_y = gamepad2.y;

        //Gamepad bumpers
        g1_right_bumper = gamepad1.right_bumper;
        g1_left_bumper = gamepad1.left_bumper;
        g2_right_bumper = gamepad2.right_bumper;
        g2_left_bumper  = gamepad2.left_bumper;

        //Gamepad triggers
        g2_right_trigger = gamepad2.right_trigger;
        g2_left_trigger = gamepad2.left_trigger;
    }
}
