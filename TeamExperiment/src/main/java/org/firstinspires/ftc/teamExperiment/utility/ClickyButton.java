package org.firstinspires.ftc.teamExperiment.utility;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by clint on 12/16/16.
 */

public class ClickyButton {


    private boolean lastCheckedValue = false;

    private GamepadButton button = null;

    private Gamepad gamePad= null;

    public ClickyButton(Gamepad gp, GamepadButton b){
        gamePad = gp;
        button = b;
    }

    /**
     * <B>THIS WILL ONLY REPLY CORRECTLY THE FIRST TIME AFTER A STATE SWITCH</B>
     *
     * @return true if the current value is different that that last time this method was called.
     */
    private boolean didButtonSwitch(){
        boolean returnValue = false;
        boolean buttonValue = readButton(this.gamePad, this.button);
        if ( buttonValue != lastCheckedValue ){
            returnValue = true;
            lastCheckedValue = buttonValue;
        }
        return returnValue;
    }

    public boolean isButtonDown(){
        return readButton(this.gamePad, this.button);
    }

    public boolean buttonClicked(){
        return didButtonSwitch() && !isButtonDown();
    }



    public static boolean readButton(Gamepad gp, GamepadButton button){

        boolean returnValue = false;
        switch(button) {

            case A:
                returnValue = gp.a;
                break;
            case B:
                returnValue = gp.b;
                break;
            case X:
                returnValue = gp.x;
                break;
            case Y:
                returnValue = gp.y;
                break;
            case DPAD_LEFT:
                returnValue = gp.dpad_left;
                break;
            case DPAD_RIGHT:
                returnValue = gp.dpad_right;
                break;
            case DPAD_UP:
                returnValue = gp.dpad_up;
                break;
            case DPAD_DOWN:
                returnValue = gp.dpad_down;
                break;
            case LEFT_STICK_BUTTON:
                returnValue = gp.left_stick_button;
                break;
            case RIGHT_STICK_BUTTON:
                returnValue = gp.right_stick_button;
                break;
            case LEFT_BUMPER:
                returnValue = gp.left_bumper;
                break;
            case RIGHT_BUMPER:
                returnValue = gp.right_bumper;
                break;
            case START:
                returnValue = gp.start;
                break;
            case BACK:
                returnValue = gp.back;
                break;
            default:
                throw new IllegalArgumentException("Unknown GamepadButton enum");
        }
        return returnValue;
    }



}
