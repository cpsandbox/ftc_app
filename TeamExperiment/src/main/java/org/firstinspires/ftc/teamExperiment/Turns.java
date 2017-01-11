package org.firstinspires.ftc.teamExperiment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamExperiment.utility.ClickyButton;
import org.firstinspires.ftc.teamExperiment.utility.GamepadButton;

/**
 * Created by clint on 1/9/17.
 */
@TeleOp(name="Turns")
public class Turns extends BaseLinearOp {

    @Override
    public void runOpMode() throws InterruptedException {
        super.initHardware();

        super.waitForStart();

        ClickyButton X = new ClickyButton(gamepad1, GamepadButton.X);
        ClickyButton B = new ClickyButton(gamepad1, GamepadButton.B);
        ClickyButton A = new ClickyButton(gamepad1, GamepadButton.A);

        while(super.opModeIsActive()){

            if( X.buttonClicked()){
                telemetry.addLine("X");
                telemetry.update();
                super.turnIMU(90);
            }

            if( B.buttonClicked()){
                telemetry.addLine("B");
                telemetry.update();
                super.turnIMU(-90);
            }

            if( A.buttonClicked()){
                telemetry.addLine("A");
                telemetry.update();
                super.turnIMU(180);
            }


        }

    }
}
