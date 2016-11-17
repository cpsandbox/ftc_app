package org.firstinspires.ftc.team9374;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Created by darwin on 10/29/16.
 */
@TeleOp(name = "9KT", group = "null")
//@Disabled0
public class NineK_MainBotT extends OpMode {
    //Drivin
    DcMotor left_f;
    DcMotor right_f;
    DcMotor left_b;
    DcMotor right_b;
    //Shooter
    DcMotor shooter_l;
    DcMotor shooter_r;
    //Speeds
    boolean Sspeed;
    boolean Nspeed = true;
    boolean Fspeed;

    Servo elevator;

    public ElapsedTime runTime = new ElapsedTime();

    public void init()  {
        //Driving motors
        left_f = hardwareMap.dcMotor.get("Eng1-left");
        right_f = hardwareMap.dcMotor.get("Eng1-right");
        left_b = hardwareMap.dcMotor.get("Eng2-left");
        right_b = hardwareMap.dcMotor.get("Eng2-right");
        //Shooter motors
        shooter_r = hardwareMap.dcMotor.get("Eng3-left");
        shooter_l = hardwareMap.dcMotor.get("Eng3-right");

        elevator = hardwareMap.servo.get("Ser1-center");

        //This might not be true for all motors on the right side
        right_b.setDirection(DcMotorSimple.Direction.REVERSE);
        right_f.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter_r.setDirection(DcMotorSimple.Direction.REVERSE);

        runTime.reset();
    }

    @Override
    public void loop() {
        //All Driving code//


        float leftDC = gamepad1.left_stick_y;
        float rightDC =  gamepad1.right_stick_y;

        double rTrigger = gamepad2.right_trigger;
        double lTrigger = gamepad2.left_trigger;

        boolean lBumper = gamepad2.left_bumper;
        boolean rBumper = gamepad2.right_bumper;
        //left.setDirection(DcMotorSimple.Direction.REVERSE);//Or .FORWARD

        if (gamepad1.right_stick_button) {
            //Normal speed, fast speed and slow speed
            Sspeed = false;
            Fspeed = true;
            Nspeed = false;
        }
        if (gamepad1.left_stick_button) {
            Fspeed = false;
            Sspeed = true;
            Nspeed = false;
        }
        if (gamepad1.y){
            Sspeed = false;
            Fspeed = false;
            Nspeed = true;
        }
        if (Sspeed){
            left_b.setPower(rightDC*.10);
            left_f.setPower(rightDC*.10);
            right_b.setPower(leftDC*.10);
            right_f.setPower(leftDC*.10);
        }
        if (Fspeed){//
            left_f.setPower(rightDC*.99);
            left_b.setPower(rightDC*.99);
            right_f.setPower(leftDC*.99);
            right_b.setPower(leftDC*.99);
        }
        if (Nspeed){//changed
            left_f.setPower(rightDC*.66);
            left_b.setPower(rightDC*.66);
            right_b.setPower(leftDC*.66);
            right_f.setPower(leftDC*.66);
        }
        //End of Driving code//
        //All Servo Code//

        rTrigger = Range.clip(rTrigger,0.5,1); //Controlls right side
        lTrigger = lTrigger/-5 + 0.5;          //Controlls left side

        if (rTrigger > 0.5){
            elevator.setPosition(rTrigger);
        } else if (lTrigger > 0){
            elevator.setPosition(lTrigger);
        } else
        //Shooter code
        if (lBumper) {
            shooter_l.setPower(1);
        } else {
            shooter_l.setPower(0);
        }
        if (rBumper) {
            shooter_r.setPower(1);
        } else {
            shooter_r.setPower(0);
        }
        telemetry.addData("Right Trigger",rTrigger);
        telemetry.addData("Left Trigger", lTrigger);
        telemetry.addData("Servo Position", elevator.getPosition());
        telemetry.addData("Total Runtime:", runTime);



    }
}
