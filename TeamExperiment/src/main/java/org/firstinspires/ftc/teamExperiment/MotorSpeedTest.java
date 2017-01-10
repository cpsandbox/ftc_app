package org.firstinspires.ftc.teamExperiment;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamExperiment.utility.BoxFilterMotorSpeed;
import org.firstinspires.ftc.teamExperiment.utility.MotorSpeed;
import org.firstinspires.ftc.teamExperiment.utility.TickLog;

/**
 * Created by clint on 12/9/16.
 */
@TeleOp(name="Motor Speed Test")
public class MotorSpeedTest extends BaseLinearOp {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initDriveMotors();
        BoxFilterMotorSpeed motorSpeed = new BoxFilterMotorSpeed(10,super.getDriveMotors());
        TickLog tl = new TickLog(super.getDriveMotors());

        super.waitForStart();
        long lastLoop = System.nanoTime();
        long ct = System.nanoTime();
        while(super.opModeIsActive()){


            squaredTankDrive(super.gamepad1.left_stick_y, super.gamepad1.right_stick_y);

            motorSpeed.read();
            tl.read();
            double[] speeds = motorSpeed.getSpeeds();

            for (int i =0; i < speeds.length; i++) {
                telemetry.addData("m"+i, String.format("%.2f",speeds[i]));
            }
            ct = System.nanoTime();

            telemetry.addData("DT (ns) ", ct - lastLoop);
            telemetry.update();
            lastLoop = ct;
        }
        setMotorPower(0,getDriveMotors());
        tl.writeData("TickLog.tsv");
       // stop();
    }
}
