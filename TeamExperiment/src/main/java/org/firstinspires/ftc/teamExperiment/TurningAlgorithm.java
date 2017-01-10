package org.firstinspires.ftc.teamExperiment;

import junit.framework.Assert;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by clint on 12/3/16.
 * <pre>
 * Goal take an arbitrary turning angle where,
 * where angles increase counter clockwise,
 * and regardless of current heading calculate the
 * correct turn regardless of current heading and
 * the possibility of crossing the +- 180 line
 *
 * Assume that heading is in  [-180,+180) range
 * Yes -180, no 180
 *
 * (this may not be easy)
 * The turning angle is in the range (-360,360)
 * Turns greater than +- 360 will be corrected
 * to the remainder of +-360
 *
 * We want the bot to be able to execute a 360ish spin
 *
 * examples:
 * <h2>Easy:</h2>
 * <pre>
 * InitialHeading:    0
 * Requested angle: -90
 * Target angle:    -90
 * The -90 indicates that a clockwise turn of 90 degrees was requested
 * (If 270 is requested the end result is the same position but the request was for a counter-clockwise turn)
 *
 * If current heading is 10 then
 * delta should be -100
 * If current heading is -10 then
 * delta should be -80
 *
 * If the robot is bumped then is is possible for |delta| > 360
 *
 * <h2>Harder:</h2>
 *
 * <pre>
 * InitialHeading: -170
 * Requested angle: -90
 * Target angle:    100
 *
 * If current heading is -170
 * delta should be        -90
 *
 * if current heading is -180
 * delta should be       -80
 *
 * if current heading is    170
 * delta should be        -70
 *
 *
 * <h2>Thinking</h2>
 * <pre>
 * You can't a priori figure out which way to
 * turn if the turn is greater than +-180
 *
 * A turn greater than +- 180 looks like the
 * robot went passed its target angle and
 * should reverse turn direction.
 *
 * If there is a hiccup that would cause the robot
 * to exceed a delta of +-180 (like getting
 * pushed or bumped) then if would assume that
 * it went passed the turn and try to reverse.
 *
 * So a turn greater than +-180 becomes a stateful matter.
 * A turn of +- 180 is a touchy matter because a jiggle at
 * the begining of the turn may send it in the wrong direction.
 *
 * So, maybe the turn is a series of checkpointed 90 degree
 * segments + remainder of 90
 *
 * makes ramping speed of turn interesting
 *
 * so a -360 turn would be
 * initial Heading+ array of checkpoints
 *
 * examples:
 * Initial angle: 0
 * requested turn: -360
 * calculated checkpoints: [-90, 180, 90, 0]
 *
 * results in
 * initial heading: 0
 * target: -90
 * intiial heading: -90
 * target: 180
 * intial heading: 180
 * target: 90
 * initial heading: 90
 * target: 0
 *
 * Then assuming that there is no catastrophic
 * interference the final turn should be able
 * to ramp down and use regular delta calculations
 * to finalize the heading
 *
 * Basically don't try to turn greater than 180,
 * so lets pick 90 as a nice number for breaking
 * up the turns
 */
public class TurningAlgorithm {

    public static void main(String[] args) {
        //testCorrect360();
        testCalculateDelta();
    }


    public static double correct360(double angle) {
        return angle % 360;
    }

    public static double normalize(double angle) {
        return AngleUnit.normalizeDegrees(angle);
    }



    /**
     * This works for angles where the difference less than +-180. The smaller of
     * the two angles is returned
     *
     * @param targetHeading  The heading at the start of the turn
     * @param currentHeading The current heading
     * @return The sign and magnitude of the angle needed to turn
     */
    public static double calculateDelta(double targetHeading, double currentHeading) {
        double ih = normalize(targetHeading);
        double ch = normalize(currentHeading);

        return  normalize(ih - ch);
    }

    private static void testCalculateDelta() {
        test(-90 , calculateDelta(-90, 0));
        test(90 , calculateDelta(90, 0));

        test(-10 , calculateDelta(-90, -80));
        test(10 , calculateDelta(-90, -100));

        test(10 , calculateDelta(90, 80));
        test(-10 , calculateDelta(90, 100));

        test(-90 , calculateDelta(-90, 0));
        test(90 , calculateDelta(90, 0));

        test(-90 , calculateDelta(100, -170));
        test(-10 , calculateDelta(180, -170));

        test(90 , calculateDelta(-100, 170));
        test(10 , calculateDelta(-180, 170));

        test(-3, calculateDelta(179, -178));
        test(-2, calculateDelta(179, -179));
        test(-1, calculateDelta(179, -180));
        test(0, calculateDelta(179, 179));

        test(3, calculateDelta(-179, 178));
        test(2, calculateDelta(-179, 179));
        test(1, calculateDelta(-179, 180));
        test(0, calculateDelta(-179, -179));

        test(-1, calculateDelta(180, -179));
        test(-1, calculateDelta(-180, -179));
        test(1, calculateDelta(180, 179));
        test(1, calculateDelta(-180, 179));

        test(0, calculateDelta(180, -180));
    }

    private static void test( double expect, double test){
        test(expect, test, .0000000001);
    }

    private static void test( double expect, double test, double maxDifference){
        double diff = expect - test;
        boolean fail =  Math.abs(diff) > maxDifference ;
        if ( fail ){
            System.out.println( expect + " != " + test + " ** fail" );
            System.out.flush();
        } else {

            System.out.println(expect + " == " + test + " pass");
            System.out.flush();
        }

    }


    private static void testCorrect360() {
        System.out.println(1 == correct360(361));
        System.out.println(-1 == correct360(-361));
        System.out.println(359 == correct360(359));
        System.out.println(-359 == correct360(-359));

        System.out.println(359.999 == correct360(359.999));
        System.out.println(-359.999 == correct360(-359.999));

        System.out.println(0 == correct360(360));
        System.out.println(0 == correct360(360));
    }


}
