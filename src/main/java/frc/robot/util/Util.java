package frc.robot.util;

public class Util {

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /**
    * Get distance to target from elevation using trigonometry
    * @param camHeight distance from ground to camera aperture
    * @param camElevation angle of camera
    * @param basketHeight distance from ground to basket vision targets
    * @param targetElevation angle of target relative to camera
    * @return distance to target
     */
    public static double findDist(double camHeight, double camElevation, double basketHeight, double targetElevation) {
        return ((basketHeight - camHeight) / Math.tan(Math.toRadians(targetElevation + camElevation)));
    }

}
