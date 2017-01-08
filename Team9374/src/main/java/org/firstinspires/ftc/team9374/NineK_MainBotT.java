package org.firstinspires.ftc.team9374;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
    Hardware9374 robot = new Hardware9374();
    public ElapsedTime runTime = new ElapsedTime();

    double gear = 1;

    public void init()  {
        //Driving motors
    robot.init(hardwareMap,telemetry);

    }

    @Override
    public void loop() {
        //All Driving code//
        //Driver

        double lStickY = -gamepad1.left_stick_y;
        double rStickY = -gamepad1.right_stick_y;

        //Operator
        double rTrigger = gamepad2.right_trigger;
        double lTrigger = gamepad2.left_trigger;
        boolean lBumper = gamepad2.left_bumper;
        boolean rBumper = gamepad2.right_bumper;



        boolean A = gamepad1.a;
        boolean B = gamepad1.b;
        boolean Y = gamepad1.y;

        //left.setDirection(DcMotorSimple.Direction.REVERSE);//Or .FORWARD
        //Mecahnim wheels

        //End of Mechanim wheel
        //Controlls right side

        //RTrigger is same, 0 and 1.
        lTrigger = lTrigger*-1;
        //Should default to 0 and 1

        //Only thig is that this assumes ltrugger is less than 1, which it should be
        if (A) {            //A is back to normal speed
            gear = 1;
            //Just for ease of controll. Not have to go back and forth.
        } else if (B) {     //B is 2nd to last gear
            gear = .3;
        } else if (Y) {
            gear = .2;      //Y is last gear, or slowest strength.
        }
        //------Verible driving-------//

        robot.left_f.setPower(lStickY*gear);
        robot.left_b.setPower(lStickY*gear);

        robot.right_f.setPower(rStickY*gear);
        robot.right_b.setPower(rStickY*gear);

        //------Verible driving-------//




        robot.elevator.setPower(gamepad2.left_stick_y);
        //Shooter code

        if (lBumper)  {
            robot.shooter_l.setPower(1);
        } else {
            robot.shooter_l.setPower(0);
        }

        if (rBumper)   {
            robot.shooter_r.setPower(1);
        } else {
            robot.shooter_r.setPower(0);
        }
        telemetry.addData("Elevator", robot.elevator.getPower());
        telemetry.addData("Right trigger", rTrigger);
        telemetry.addData("Left trigger", lTrigger);
        telemetry.addData("Robot Red:",robot.CSensor.red());
        telemetry.addData("Robot blue",robot.CSensor.blue());
        telemetry.addData("Heading", robot.getcurrentheading());

    }
}
