package org.firstinspires.ftc.team9374;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by darwin on 12/17/16.
 */
@Autonomous(name = "NineK_MainBotA_Beacon_BV2")
public class NineK_MainBotA_Beacon_B extends LinearOpMode {
    Hardware9374 robot = new Hardware9374();

    @Override public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (super.opModeIsActive()){

            robot.moveToPosition(48,.2);

            robot.resetEncoders();

            robot.Turn(315,.4);

            robot.resetEncoders();

            robot.moveToPosition(48,.3);

            robot.resetEncoders();

            while (true) {
                telemetry.addLine("Made it to loop");
                if (robot.CSensor.blue() > 4) {
                    robot.moveToPosition(-5, .4);
                    robot.resetEncoders();
                    break;
                } else if (robot.CSensor.red() > 4) {
                    robot.moveToPosition(-5, .4);
                    robot.resetEncoders();

                    robot.moveToPosition(5, 1);
                    robot.resetEncoders();
                }
            }

            break;
        }
    }
}
