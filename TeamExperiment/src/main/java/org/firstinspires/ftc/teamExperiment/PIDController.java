package org.firstinspires.ftc.teamExperiment;


/**
 * Proportional &lt;Kp&gt;, Integral &lt;Ki&gt;, Derivitive &lt;Kd&gt; controller implementation.
 * <p>
 * Kp depends on the present error, Ki on the accumulation of past errors, and Kd is a prediction of future errors, based on
 * current rate of change.
 * <p> <b>Proportional gain, Kp:</b>
 * Larger values typically mean faster response since the larger the error, the larger the proportional term compensation.
 * An excessively large proportional gain will lead to process instability and oscillation.<br>
 * <p><b>Integral gain, Ki:</b>
 * Larger values imply steady state errors are eliminated more quickly. The trade-off is larger overshoot: any negative error integrated during transient response must be integrated away by positive error before reaching steady state.
 * <p><b>Derivative gain, Kd:</b>
 * Larger values decrease overshoot, but slow down transient response and may lead to instability due to signal noise amplification in the differentiation of the error.
 * <p><b>Definitions:</b>
 * <ul>
 * <li>MV - Manipulated Variable. What the PID contoller calculates to be used as the input to the process (i.e. motor speed).
 * <LI>PV - Process Variable. The measurement of the process value.
 * <LI>SP - Setpoint. The desired value that the PID contoller works to.
 * <li>e - Error. The difference between PV and MV
 * </ul>
 * <p>
 * The proportional term Kp (sometimes called gain) makes a change to the output that is proportional to the
 * current error value. The proportional response is adjusted by multiplying the error by Kp. A high proportional gain
 * results in a large change in the output for a given change in the error.
 * <p>
 * The integral term Ki (sometimes called reset) accelerates the movement of the process towards the setpoint and eliminates the residual
 * steady-state error that
 * occurs with a proportional only controller. However, since the integral term is responding to accumulated errors from the past,
 * it can cause the present value to overshoot the setpoint value.
 * <p>
 * The derivative term Kd (sometimes called rate) slows the rate of change of the controller output and this effect is most noticeable
 * close to the controller setpoint. Hence, derivative control is used to reduce the magnitude of the overshoot produced by the
 * integral component (Ki) and improve the combined controller-process stability.
 * <p>
 * It is important to tune the PID controller with an implementation of a consistent delay between calls to <code>doPID()</code>
 * because the MV calc in a PID controller is time-dependent by definition. The implementation calculates the time delta between
 * calls to <code>{@link #doPID}</code>.
 *
 * @auther Kirk Thompson, 2/5/2011 lejos[AT]mosen[dot]net
 */
public class PIDController {
    /**
     * Proportional term ID
     *
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_KP = 0;
    /**
     * Integral term ID
     *
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_KI = 1;
    /**
     * Derivitive term ID
     *
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_KD = 2;
    /**
     * The Ramping Exponential value ID. Used for output ramping which determines ramp shape. 1.0=linear, Set to 0 to disable
     * output ramping. Larger values create steeper ramping curves.
     * steeper ramps.
     *
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_RAMP_POWER = 3;
    /**
     * The Ramping Threshold value ID. Used for output ramping. When the PID Manipulated Variable (MV) is within this range (-+), output ramping is
     * applied to MV before it is returned from <code>{@link #doPID}</code>.
     *
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_RAMP_THRESHOLD = 4;
    /**
     * The deadband value ID. Used for output clipping. if MV within +- this range relative to zero, MV of zero is returned.
     * Set to zero to effectively disable. This is useful to avoid hunting arount the SP when there is a lot
     * of slop in whatever the controller is controlling i.e. gear & link lash.
     *
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_DEADBAND = 5;

    /**
     * The high limit cutoff value ID. Use for  high limit cutoff for Manipulated Variable (MV). Set to a large value to
     * effectively disable. This is applied to MV before any ramping. Default is 900.
     * at instantiation.
     *
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_LIMITHIGH = 6;

    /**
     * The low limit cutoff value ID. Use for  low limit cutoff for Manipulated Variable (MV). Set to a large negative value to
     * effectively disable. This is applied to MV before any ramping. Default is -900.
     *
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_LIMITLOW = 7;

    /**
     * The Setpoint value ID. This is the value the PID controller works toward.
     *
     * @see #setPIDParam
     * @see #getPIDParam
     */
    public static final int PID_SETPOINT = 8;

    //private BTLogger dataLogger = null;     // the datalogger instance that may be registered
    //private boolean hasDataLogger = false;  // If a datalogger has been registered, this is set to true
    // Our default Constants for the PID controller
    private float Kp = 1.8f;          // proportional value determines the reaction to the current error
    private float Ki = 0.00001f;      // integral value determines the reaction based on the sum of recent errors
    private float Kd = 150.0f;        // derivative value determines the reaction based on the rate at which the error has been changing
    private int highLimit = 900;            // assuming control of motor speed and thereby max would be 900 deg/sec
    private int lowLimit = -highLimit;
    private int previous_error = 0;
    private int deadband = 0;

    private long logBeginTime;              // baseline for logging absolute time
    private int dt = 0;                     // cycle time, ms
    private long cycleTime = 0;               // used to calc the time between each call (dt) to doPID()
    private int setpoint;                   // The setpoint to strive for
    private int error;                      // proportional term
    private int integral = 0;               // integral term
    private float derivative;               // derivitive term

    private float power = 0;
    private int rampThresold = 0;
    private double rampExtent = 1;

    /**
     * construct a PID controller instance using passed setpoint (SP)
     *
     * @param setpoint The goal of the MV
     */
    public PIDController(int setpoint) {
        logBeginTime = System.currentTimeMillis();
        this.setpoint = setpoint;
    }

    /**
     * Set PID controller parameters.
     *
     * @param paramID What parameter to set. See the constant definitions for this class.
     * @param value   The value to set it to
     * @see #getPIDParam
     */
    public void setPIDParam(int paramID, float value) {
        switch (paramID) {
            case PIDController.PID_KP:
                this.Kp = value;
                break;
            case PIDController.PID_KI:
                this.Ki = value;
                break;
            case PIDController.PID_KD:
                this.Kd = value;
                break;
            case PIDController.PID_RAMP_POWER:
                this.power = value;
                rampExtent = Math.pow(this.rampThresold, this.power);
                break;
            case PIDController.PID_RAMP_THRESHOLD:
                this.rampThresold = (int) value;
                if (this.rampThresold == 0) break;
                rampExtent = Math.pow(this.rampThresold, this.power);
                break;
            case PIDController.PID_DEADBAND:
                this.deadband = (int) value;
                break;
            case PIDController.PID_LIMITHIGH:
                this.highLimit = (int) value;
                break;
            case PIDController.PID_LIMITLOW:
                this.lowLimit = (int) value;
                break;
            case PIDController.PID_SETPOINT:
                this.setpoint = (int) value;
                cycleTime = 0;
                integral = 0;
                break;
            default:
        }
    }

    /**
     * Get PID controller parameters.
     *
     * @param paramID What parameter to get. See the constant definitions for this class.
     * @return The requested parameter value
     * @see #setPIDParam
     */
    public float getPIDParam(int paramID) {
        float retval = 0.0f;
        switch (paramID) {
            case PIDController.PID_KP:
                retval = this.Kp;
                break;
            case PIDController.PID_KI:
                retval = this.Ki;
                break;
            case PIDController.PID_KD:
                retval = this.Kd;
                break;
            case PIDController.PID_RAMP_POWER:
                retval = this.power;
                break;
            case PIDController.PID_RAMP_THRESHOLD:
                retval = this.rampThresold;
                break;
            case PIDController.PID_DEADBAND:
                retval = this.deadband;
                break;
            case PIDController.PID_LIMITHIGH:
                retval = this.highLimit;
                break;
            case PIDController.PID_LIMITLOW:
                retval = this.lowLimit;
                break;
            case PIDController.PID_SETPOINT:
                retval = this.setpoint;
                break;
            default:
        }
        return retval;
    }

    /**
     * Do the PID calc for a single iteration. Your implementation must provide the delay between calls to this method .
     *
     * @param processVariable The PV value from the process (sensor reading, etc.).
     * @return The output Manipulated Variable (MV)
     */
    public int doPID(int processVariable) {
        int outputMV;
//        LCD.drawString("PV: " + processVariable + "  " , 0, 5);
//        if (true) return 0;
        if (cycleTime == 0) {
            cycleTime = System.currentTimeMillis();
            return 0;
        }
        error = setpoint - processVariable;
        error = Math.abs(error) <= deadband ? 0 : error;
        integral += error * dt; // error to time product
        derivative = ((float) (error - previous_error)) / dt;
        outputMV = (int) (Kp * error + Ki * integral + Kd * derivative);

        if (outputMV > highLimit) outputMV = highLimit;
        if (outputMV < lowLimit) outputMV = lowLimit;
        previous_error = error;

        //LCD.drawString("dt=" + dt + " ", 9, 5); // TODO remove after debugging/testing

        dt = (int) (System.currentTimeMillis() - cycleTime);

        cycleTime = System.currentTimeMillis();
        return rampOut(outputMV);
    }

    private int rampOut(int ov) {
        if (power == 0 || rampThresold == 0) return ov;
        if (Math.abs(ov) > rampThresold) return ov;
        int workingOV;
        workingOV = (int) (Math.pow(Math.abs(ov), power) / rampExtent * rampThresold);
        return (ov < 0) ? -1 * workingOV : workingOV;
    }
}



