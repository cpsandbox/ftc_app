package org.firstinspires.ftc.team9374;

/**
 * Created by darwin on 11/30/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by darwin on 9/20/16.
 */

@TeleOp(name="Mecahnim_Drive", group="Normal_Opmode")

public class Mecanum_Function_ArcadeD extends OpMode {
    DcMotor left_f;
    DcMotor right_f;
    DcMotor left_b;
    DcMotor right_b;
    String mode;
    //Controller vaibles
    double lStickY;
    double lStickX;
    double rStickY;
    //Power varibles
    double LFpower;
    double RFpower;
    double LBpower;
    double RBpower;
    public void init()  {
        mode = "tank";
        left_f = hardwareMap.dcMotor.get("Eng1-left");
        right_f = hardwareMap.dcMotor.get("Eng1-right");
        left_b = hardwareMap.dcMotor.get("Eng2-left");
        right_b = hardwareMap.dcMotor.get("Eng2-right");
    }

    public void loop() {

        lStickY = gamepad1.left_stick_y;
        lStickX = gamepad1.left_stick_x;
        rStickY = gamepad1.right_stick_y;

        //Shooter code

        LFpower = lStickY + lStickX + rStickY;
        RFpower = lStickY + lStickX - rStickY;
        LBpower = lStickY - lStickX + rStickY;
        RBpower = lStickY - lStickX + rStickY;

        left_f.setPower(Range.clip(LFpower,-1,1));
        right_f.setPower(Range.clip(RFpower,-1,1));
        left_b.setPower(Range.clip(LBpower,-1,1));
        right_b.setPower(Range.clip(RBpower,-1,1));

    }
}
