package org.firstinspires.ftc.team9374;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Created by darwin on 10/29/16.
 */
@Autonomous(name = "9374_MAIN_AUTONOMOUS_CENTER",group = "null")

public class NineK_MainBotA extends LinearOpMode {

    Hardware9374 robot = new Hardware9374();


    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        super.waitForStart();

        while (super.opModeIsActive()) {
            /*

            //Dont quite know what this is?
            int clicks = calcClicksForInches(5);

            setALLpower(.5);

            setALLposition(clicks);

            while (opModeIsActive()) {
                telemetry.addData("Target:", clicks);
                telemetry.addData("Left Position", left_f.getCurrentPosition());
                if (left_f.getCurrentPosition() > clicks) {
                    break;
                }
            }
            */
            //Spin up the motors, then shoot after  secounds.
            while (true) {
                robot.shooter_l.setPower(1);
                robot.shooter_r.setPower(1);
                if (robot.runTime.time() > 5) {
                    robot.elevator.setPower(.2);
                    if (robot.runTime.time() > 10) {
                        break;
                    }
                }
            }
            //move 55 inches
            int clicks = robot.calcClicksForInches(55);
            //GOGOOG
            robot.setALLpower(.5);

            robot.setALLposition(clicks);
            //Printing out our
            while (opModeIsActive()) {
                telemetry.addData("Target:", clicks);
                telemetry.addData("Left Position", robot.left_f.getCurrentPosition());
                if (robot.left_f.getCurrentPosition() > clicks) {
                    break;
                }
            }
            break;
        }

    }
}
