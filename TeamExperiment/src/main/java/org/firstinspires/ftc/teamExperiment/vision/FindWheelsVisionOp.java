package org.firstinspires.ftc.teamExperiment.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamExperiment.BaseLinearOp;

/**
 * Created by clint on 12/14/16.
 */
@Autonomous(name="Wheel Seek")

public class FindWheelsVisionOp extends BaseLinearOp {
    @Override
    public void runOpMode() throws InterruptedException {

        super.initHardware();
        CRServo slider = hardwareMap.crservo.get("slider");
        TouchSensor leftStop = hardwareMap.touchSensor.get("leftLimit");

        // set up motor
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQu6WOf/////AAAAGVqnDd0rNkrbkDbHb/jGqaCDSL+Y+rEEAHojylymEOEZTc063myD4vrCEACxydJtv1p631fl5MrcL+vRaFVLYdtrubj8+0UNEE+VWPwh1SvttC5IK5SXUKrhP1c0aE1XjvYFi1s1iRt50eD2KPCpDvafRGHK0P3OHDI5MNb8BVVhkq2/hOHrkCqM3f05rjNFi0g6P/+it/k0cMkOsvqdcyYAp/32sIvCh1IslARDkHjPrsfIx0Ze67HNe1yICS/JnoMKdp34KOkPXZnaiKRDqGZCYotoXuUTnep43ghZoV24Q2TlJoo+vaa/ss+qOrms8QvgDQgqnLJIfiEUEghdlHaejH2N7WAsy42STdlkvQGn";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");
        beacons.activate();

        VuforiaTrackableDefaultListener wheelsListener = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        OpenGLMatrix pose = null;

        slider.setPower(-1);
        while (slider.getController().getPwmStatus() == ServoController.PwmStatus.ENABLED){
            if (leftStop.isPressed()){
                slider.getController().pwmDisable();
            }
        }

        super.waitForStart();

        super.setMotorPower(.3, getDriveMotors());

        while (opModeIsActive() && pose == null) {
            // drive toward beacon
            // set motors and use dead reckoning
            while (opModeIsActive() && wheelsListener.getRawPose() == null) {
                idle();
            }
            pose = wheelsListener.getPose();
        }
        super.setMotorPower(0, getDriveMotors());

        telemetry.addLine("wheels found");
        telemetry.update();
        VectorF angles = anglesFromTarget(wheelsListener);

        // change last 0 for side to side
        VectorF trans = navOffWall(pose.getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(100, 0, 0));

        telemetry.addLine("turning");
        telemetry.update();
        // turn
        if (trans.get(0) > 0) {
            super.setMotorPower(.1, getLeftMotors());
            super.setMotorPower(-.1, getRightMotors());
        } else {
            super.setMotorPower(-.1, getLeftMotors());
            super.setMotorPower(.1, getRightMotors());
        }

        do {
            if ((pose = wheelsListener.getPose()) != null) {
                trans = navOffWall(pose.getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(100, 0, 0));
            }
        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30);
        super.setMotorPower(0, getDriveMotors());
        telemetry.addLine("end turning");
        telemetry.update();
        super.setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, getDriveMotors());
        super.setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION,getDriveMotors());
        telemetry.addLine("drive straight");
        telemetry.update();
        int phoneToCenterOfTurn_mm = -150;
        double wheelCircumference_mm = WHEEL_CIRCUMFERENCE_MM;
        int ticksperRev = 1120;


        super.setMotorTarget((int) (((Math.hypot(trans.get(0), trans.get(2)) + phoneToCenterOfTurn_mm) / wheelCircumference_mm * ticksperRev)), super.getDriveMotors());

        super.setMotorPower(.2, getDriveMotors());


        while (opModeIsActive() && super.someMotorsAreBusy(super.getDriveMotors())) {
            idle();
        }
        telemetry.addLine("end drive straight");
        telemetry.update();
        super.setMotorPower(0, getDriveMotors());

        super.setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER, getDriveMotors());

        telemetry.addLine("final turn");
        telemetry.update();
        while (opModeIsActive() && ((pose = wheelsListener.getPose()) == null || Math.abs(wheelsListener.getPose().getTranslation().get(0)) > 10)) {
            if (pose != null) {
                // turn
                if (pose.getTranslation().get(0) > 0) {
                    super.setMotorPower(.1, getLeftMotors());
                    super.setMotorPower(-.1, getRightMotors());
                } else {
                    super.setMotorPower(-.1, getLeftMotors());
                    super.setMotorPower(.1, getRightMotors());
                }
            } else {
                // turn
                super.setMotorPower(.1, getLeftMotors());
                super.setMotorPower(-.1, getRightMotors());

            }
        }
        super.setMotorPower(0, getDriveMotors());

        telemetry.addLine("done");
        telemetry.update();
    }


    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall) {
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image) {
        float[] data = image.getRawPose().getData();
        float[][] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float) thetaX, (float) thetaY, (float) thetaZ);
    }
}
