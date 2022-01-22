package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "spin_carousel", group = "Knightrix")
public class spin_carousel extends LinearOpMode{

    CRServo crServo;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode(){

        crServo = hardwareMap.get(CRServo.class, "cont_Servo");

        waitForStart();
        spin(5000);

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

}
