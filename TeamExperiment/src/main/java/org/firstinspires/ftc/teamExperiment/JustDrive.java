package org.firstinspires.ftc.teamExperiment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by clint on 12/16/16.
 */
@TeleOp(name="Just Drive")
public class JustDrive extends BaseLinearOp{

    @Override
    public void runOpMode() throws InterruptedException {
        super.initDriveMotors();

        waitForStart();
        while(opModeIsActive()){
            super.tankDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }
    }
}
