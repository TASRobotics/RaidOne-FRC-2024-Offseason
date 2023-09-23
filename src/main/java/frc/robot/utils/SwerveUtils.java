package frc.robot.utils;

public class SwerveUtils {
    /**
     * Steps a value towards a target with a specified step size.
     * @param iCurrent The current or starting value.  Can be positive or negative.
     * @param iTarget The target value the algorithm will step towards.  Can be positive or negative.
     * @param iStepsize The maximum step size that can be taken.
     * @return The new value for {@code iCurrent} after performing the specified step towards the specified target.
     */
    public static double StepTowards(double iCurrent, double iTarget, double iStepsize) {
        if (Math.abs(iCurrent - iTarget) <= iStepsize) {
            return iTarget;
        }
        else if (iTarget < iCurrent) {
            return iCurrent - iStepsize;
        }
        else {
            return iCurrent + iStepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param iCurrent The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param iTarget The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param iStepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code iCurrent} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double StepTowardsCircular(double iCurrent, double iTarget, double iStepsize) {
        iCurrent = WrapAngle(iCurrent);
        iTarget = WrapAngle(iTarget);

        double stepDirection = Math.signum(iTarget - iCurrent);
        double difference = Math.abs(iCurrent - iTarget);
        
        if (difference <= iStepsize) {
            return iTarget;
        }
        else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (iCurrent + 2*Math.PI - iTarget < iStepsize || iTarget + 2*Math.PI - iCurrent < iStepsize) {
                return iTarget;
            }
            else {
                return WrapAngle(iCurrent - stepDirection * iStepsize); //this will handle wrapping gracefully
            }

        }
        else {
            return iCurrent + stepDirection * iStepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param iAngleA An angle (in radians).
     * @param iAngleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double iAngleA, double iAngleB) {
        double difference = Math.abs(iAngleA - iAngleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param iAngle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double iAngle) {
        double twoPi = 2*Math.PI;

        if (iAngle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        }
        else if (iAngle > twoPi) {
            return iAngle - twoPi*Math.floor(iAngle / twoPi);
        }
        else if (iAngle < 0.0) {
            return iAngle + twoPi*(Math.floor((-iAngle) / twoPi)+1);
        }
        else {
            return iAngle;
        }
    }
}
