package org.firstinspires.ftc.teamExperiment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by clint on 12/13/16.
 */

@TeleOp(name = "ColorSeek Blue")

public class ColorSeekBlueOpMode extends LinearOpMode {
    private CRServo slider;
    private ColorSensor mcs = null;
    private TouchSensor leftLimit = null;
    private TouchSensor rightLimit = null;

    private final double leftDirection = -1;
    private final double rightDirection = 1;

    private final double maxLeft = -1;
    private final double maxRight = 1;


    private double servoPosition = 0;


    public void initHardware() {
        slider = hardwareMap.crservo.get("slider");
        leftLimit = hardwareMap.touchSensor.get("leftLimit");
        rightLimit = hardwareMap.touchSensor.get("rightLimit");

        mcs = hardwareMap.colorSensor.get("mcs");
        //mcs.enableLed(false);

        slider.setPower(0);

    }


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        super.waitForStart();


        // start by moving left

        double targetPosition = Range.clip(maxLeft, -1, 1);
        slider.setPower(targetPosition);
        boolean searching = true;
        boolean ready = false;

        while (super.opModeIsActive()) {
            // if found moving left continue unitl hit left stop
            if (mcs.blue() > 0) {
                searching = false;
            }

            if (searching) {
                if (leftLimit.isPressed() && targetPosition < 0) {
                    targetPosition = -targetPosition;
                }

                if (rightLimit.isPressed() && targetPosition > 0) {
                    targetPosition = -targetPosition;
                }
                slider.setPower(Range.clip(targetPosition, maxLeft, maxRight));
            } else {
                if (leftLimit.isPressed() && targetPosition < 0) {
                    slider.getController().pwmDisable();
                    ready = true;
                }

                if (rightLimit.isPressed() && targetPosition > 0) {
                    slider.getController().pwmDisable();
                    ready = true;
                }
            }




            telemetry.addLine(String.format("Limits L:%d R:%d", leftLimit.isPressed() ? 1 : 0, rightLimit.isPressed() ? 1 : 0));
            telemetry.addData("maxPowers: ", "L:%.2f R:%.2f", maxRight, maxLeft);
            telemetry.addData("Color", mcs.red() + " " + mcs.green() + " " + mcs.blue());
            telemetry.addData("Target", targetPosition);
            telemetry.addData("searching", searching);

            telemetry.update();
        }
    }
}
