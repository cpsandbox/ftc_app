package org.firstinspires.ftc.teamExperiment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by clint on 12/2/16.
 */

@TeleOp(name="Servo Slide")
public class ServoSlideTest extends OpMode{

    private CRServo slider;

    private double servoPosition=0;

    public void init(){
        slider = hardwareMap.crservo.get("slider");
        slider.setPower(servoPosition);
    }

    public void loop(){


    servoPosition = gamepad1.left_stick_y;

        slider.setPower(servoPosition);
        telemetry.addData("servo position", servoPosition);
        telemetry.update();

    }
}
