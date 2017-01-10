package org.firstinspires.ftc.team8745;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by rose on 1/2/17.
 */
@TeleOp(name="8K Ball Pickup")


public class Ball_Pickup extends OpMode {

    DcMotor left_b;
    DcMotor left_f;
    DcMotor right_b;
    DcMotor right_f;
    DcMotor motor_pickup;
    public void init() {
        //right
        right_b = hardwareMap.dcMotor.get("motor-rightBACK");
        right_f = hardwareMap.dcMotor.get("motor-right");

        //left
        left_b = hardwareMap.dcMotor.get("motor-leftBACK");
        left_f = hardwareMap.dcMotor.get("motor-left");

        //pickup
        motor_pickup = hardwareMap.dcMotor.get("motor-pickup");

        //reverse mode
        left_b.setDirection(DcMotorSimple.Direction.REVERSE);
        left_f.setDirection(DcMotorSimple.Direction.REVERSE);}

    boolean wasDown = false;

    boolean slowMode = false;

    @Override
    public void loop() {
        float r_stick = gamepad1.right_stick_y;
        float l_stick = gamepad1.left_stick_y;
        double r_slow = gamepad1.right_stick_y*.5;
        double l_slow = gamepad1.left_stick_y*.5;

        boolean r_bumper2 = gamepad2.right_bumper;

        boolean down = gamepad1.dpad_down;

        boolean down_click = false;

        down_click = wasDown&&!down;

        wasDown = down;

        if (down_click){
            slowMode = !slowMode;
        }

        if (!slowMode){
            left_b.setPower(l_stick);
            left_f.setPower(l_stick);
            right_b.setPower(r_stick);
            right_f.setPower(r_stick);
        }
        else {
            right_f.setPower(r_slow);
            right_b.setPower(r_slow);
            left_f.setPower(l_slow);
            left_b.setPower(l_slow);

        }

        if (r_bumper2){
            motor_pickup.setPower(1.0);
        }
        else {
            motor_pickup.setPower(0.0);
        }

    }
}
