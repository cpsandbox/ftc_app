package org.firstinspires.ftc.team9374;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by darwin on 12/17/16.
 */
@Autonomous(name = "NineK_MainBotA_Beacon_Beta")
public class NineK_MainBotA_Beacon_Beta extends LinearOpMode {
    Hardware9374 robot = new Hardware9374();

    int foo;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.elevator.setPower(0);

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (super.opModeIsActive()) {
            //Move to center
            robot.moveToPosition(48, .2);

            robot.resetEncoders();
            //Turn to beacon
            robot.Turn(315, .4);

            robot.resetEncoders();
            //Move to beacon
            robot.moveToPosition(48, .3);

            robot.resetEncoders();

            while (true) {
                telemetry.addLine("Made it to loop");
                robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.waitNSeconds(1);
                if (robot.CSensor.blue() > 4) {

                    robot.moveToPosition(-15, -.3);

                    robot.resetEncoders();

                    robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;
                } else  {
                    robot.moveToPosition(-15, -.3);
                    robot.resetEncoders();

                    robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.moveToPosition(16, .3);
                    robot.resetEncoders();

                    robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }

            } //End of the while (true)

            //moving us in front of the 2nd beacon
            robot.Turn(0,.1);
            robot.resetEncoders();

            robot.moveToPosition(29,.3);
            robot.resetEncoders();

            robot.Turn(315,.1);
            robot.resetEncoders();

            robot.moveToPosition(16, .2);
            robot.resetEncoders();

            break;
        }
    }
}
