package org.firstinspires.ftc.teamExperiment.utility;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamExperiment.BaseLinearOp;

import java.util.Arrays;

/**
 * Created by clint on 12/15/16.
 */
@TeleOp(name="Roll 4")
public class Roll4 extends BaseLinearOp{
    @Override
    public void runOpMode() throws InterruptedException {
        super.initDriveMotors();
        waitForStart();
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, getDriveMotors());
        setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION,getDriveMotors());
        setMotorTarget(TICS_PER_REV*4, getDriveMotors());
        setMotorPower(.2,getDriveMotors());

        telemetry.addLine("starting wait");
        telemetry.update();
        while(opModeIsActive() && someMotorsAreBusy(getDriveMotors())){
            idle();
            telemetry.addData("postition", Arrays.toString(getMotorPositions(getDriveMotors())));
            telemetry.update();
        }
        telemetry.addLine("ended wait");
        setMotorPower(0,getDriveMotors());
        while(opModeIsActive()){
            idle();
            telemetry.addData("postition", Arrays.toString(getMotorPositions(getDriveMotors())));
            telemetry.update();
        }



    }
}
