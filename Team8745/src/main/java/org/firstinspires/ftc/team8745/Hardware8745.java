package org.firstinspires.ftc.team8745;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
/**
 * Created by houserm on 12/17/16.
 */

public class Hardware8745 {



    /**
     * Created by darwin on 12/3/16.
     *
     * 9374 Robot class.
     *
     * Here is a reference for how to une this in place of other things.
     *
     *      Hardware9374 robot = new Hardware9374();     // Use a 9K' shardware
     *
     *
     *      robot.init(hardwareMap);
     *
     */

        DcMotor left_f;
        DcMotor right_f;
        DcMotor left_b;
        DcMotor right_b;
        //Shooter
        DcMotor shooter_l;
        DcMotor shooter_r;
        //Speeds
        boolean Sspeed;
        boolean Nspeed = true;
        boolean Fspeed;

        Servo shooterServo;

        ColorSensor CSensor;

        //Controller vaibles
        double lStickY;
        double lStickX;
        double rStickY;
        //Power varibles
        double LFpower;
        double RFpower;
        double LBpower;
        double RBpower;

        final double wheelDiameterInInches = 4;
        final int tpr = 1120;
        final double wheelCorrection = 0;
        //Not yet defined, will be.

        int ticks;
        final double kServoNullPosition = 0.8;
        final double kServoRange = 0.6;
        final double kShooterEnginePower = 0.7;
        BNO055IMU imu;

        public ElapsedTime runTime = new ElapsedTime();

        /* constructor, used when ... = new Hardware9374()  */
        public Hardware8745() {
        }
        //Our init, cannot be called inside the begginning because it is finding our devices.
        public void init(HardwareMap hardwareMap) {

            //Driving motors
            left_f = hardwareMap.dcMotor.get("motor-left");
            right_f = hardwareMap.dcMotor.get("motor-right");

            left_b = hardwareMap.dcMotor.get("motor-leftBack");
            right_b = hardwareMap.dcMotor.get("motor-rightBack");
            //Shooter motors
            shooter_r = hardwareMap.dcMotor.get("shooter-right");
            shooter_l = hardwareMap.dcMotor.get("shooter-left");

            shooterServo = hardwareMap.servo.get("shooter-servo");

            CSensor = hardwareMap.colorSensor.get("Col1-right");

            CSensor.enableLed(false);

            //center = hardwareMap.servo.get("Ser1-center");

            //Might be deprecated
            left_b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            left_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        left_b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
            right_f.setDirection(DcMotorSimple.Direction.REVERSE);
            right_b.setDirection(DcMotorSimple.Direction.REVERSE);

            //shooter_l.setDirection(DcMotorSimple.Direction.REVERSE);

            shooter_r.setDirection(DcMotorSimple.Direction.REVERSE);

            for (int i = 1; i <= 3; i++) {
                waitNSeconds(1);
                shooterServo.setPosition((kServoNullPosition + (-kServoRange)));
                waitNSeconds(1);
                shooterServo.setPosition(kServoNullPosition);
                waitNSeconds(1);
                shooterServo.setPosition((kServoNullPosition + (-kServoRange)));
                waitNSeconds(1);
                shooterServo.setPosition(kServoNullPosition);


                runTime.reset();

                //left.setDirection(DcMotorSimple.Direction.REVERSE);//Or .FORWARD
                //--------------------------------------------------------------------------------------
                //End of Robot init code.
                //--------------------------------------------------------------------------------------

                // Set up the parameters with which we will use our IMU. Note that integration
                // algorithm here just reports accelerations to the logcat log; it doesn't actually
                // provide positional information.

                //--------------------------------------------------------------------------------------
                //IMU code
                //--------------------------------------------------------------------------------------
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;               // Defining units
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;  // Defining units
                parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled = true;
                parameters.loggingTag = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                // and named "imu".
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);

                // Set up our telemetry dashboard

                runTime.reset();

            }
        }
        //-------------------------------------------------------
        //                      Action Functions
        //-------------------------------------------------------



        public void Turn(int THeading, double speed) {
            //Stands for THeading
            //EX: change = 90 and speed is 0.
        /*
        I am acutally really proud of myself for this method.
        This method moves the robot a certain amount of degrees.
        //-------------------------
        //True  = Counter-Clockwise
        //False = Clockwise
        //-------------------------
        */
            //everything needs to be in integers.

            //This took a lot of time to come up with one number
            //Just saying.

            left_b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_f.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double heading = getcurrentheading(); //38

            double target  = heading + THeading;

            //Making shure that the target is not over 360
            if (target > 360) {
                target = target - 360;
            }

            while (true) {
                //Not going to bother to do this logic
                left_b.setPower(speed);
                left_f.setPower(speed);
                right_b.setPower(-speed);
                right_f.setPower(-speed);
                //Heading will be in 160
                heading = getcurrentheading();
                if (heading < target + 5 && heading > target -5) { //Should be withen 10 of the target.
                    break;
                }
            }

            left_b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        public int calcClicksForInches(double distanceInInches) {
            //Currently there are 1120 different positions on any given wheel
            double revlutions = distanceInInches / (wheelDiameterInInches * Math.PI); //Find out how many revolutations
            int clicks = (int) (revlutions * tpr); //This is a pretty big number, gonna be in the 1,000's
            return clicks; //The position to set the wheels to.
        }
        public void moveToPosition(double distanceInIN,double power) {
            setALLposition(calcClicksForInches(distanceInIN));
            setALLpower(power);
            while (true) {
                if (calcClicksForInches(distanceInIN) < 0)
                    if (left_f.getCurrentPosition() < calcClicksForInches(distanceInIN)) {
                        resetEncoders();
                        setALLpower(0);

                        break;
                    }
                if (calcClicksForInches(distanceInIN) > 0)
                    if (left_f.getCurrentPosition() > calcClicksForInches(distanceInIN)) {
                        resetEncoders();
                        setALLpower(0);

                        break;
                    }
            }
        }
        public void translate(boolean direction, double power, int inches){
            //Currently not finished, need to confirm with camden.
            //-------------------------
            //True  = Left
            //False = Right
            //-------------------------
        /*
        Diagram!!!
          |  _ ________ _   ^
          | | |        | |  |
          v |_|        |_|  |
              |        |
              |        |
          ^  _|        |_
          | | |        | |  |
          | |_|________|_|  |
                            v
         So, to go left the two wheels on the left turn in
         and the two wheels on the right turn outward
         as shown.
         For right it is almost the exact same, except the right wheels turn in and the left wheels turn out
          ^  _ ________ _
          | | |        | | |
          | |_|        |_| |
              |        |   v
              |        |
             _|        |_  ^
          | | |        | | |
          | |_|________|_| |
          v
         */
            //Acutall code will continue

            setALLposition(calcClicksForInches(inches));
            if (direction) {
                // If ( left ),
                left_f.setPower(-power);
                left_b.setPower(power);
                right_f.setPower(power);
                right_b.setPower(-power);
            } else if (!direction) {
                left_f.setPower(power);
                left_b.setPower(-power);
                right_f.setPower(-power);
                right_b.setPower(power);
            }
            while (true) {
                if (left_f.getCurrentPosition() > calcClicksForInches(inches)){
                    resetEncoders();
                    break;
                }
            }


        }
        public void setALLpower(double power){
            left_b.setPower(power);
            left_f.setPower(power);
            right_b.setPower(power);
            right_f.setPower(power);
        }
        public void setALLposition(int position) {
            left_b.setTargetPosition(position);
            left_f.setTargetPosition(position);
            right_b.setTargetPosition(position);
            right_f.setTargetPosition(position);

        }
        public void resetEncoders(){
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public double getcurrentheading() {
            double angles;
            angles = AngleUnit.DEGREES.normalize(imu.getAngularOrientation()
                    .toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle);
            //If its negetive then subtract
            if (angles < 0){
                angles = 360 + angles;
            }
            return angles;

        }

        public void setMode(DcMotor.RunMode mode){
            left_b.setMode(mode);
            left_f.setMode(mode);
            right_b.setMode(mode);
            right_f.setMode(mode);
        }
        public void waitNSeconds(int secondsToWait){
            double startTime = runTime.time();
            while (runTime.time() - startTime< secondsToWait){

            }
        }

    }


