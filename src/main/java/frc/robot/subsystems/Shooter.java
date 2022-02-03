package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private double tv, tx, ty, ta;
    private double mounting_angle;

    //For the shooter. Given what the limelight sees and the shooter angle, compute the desired initial speed for the shooter.
    public double computeSpeed(double angle, double cameraHeight, double objectHeight) {
        double distance = determineObjectDist(cameraHeight, objectHeight);
        return Math.sqrt((16.1 * Math.pow(distance, 2)) / (distance * Math.tan(angle) - cameraHeight - objectHeight)) / Math.cos(angle);
      }

    /* Determine the distance an object is from the limelight given the camera's height 
    off of the ground and the object's height off of the ground. */
    public double determineObjectDist(double cameraHeight, double objectHeight) {
        System.out.println("Distance: ");
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        //0.36 is mounting angle
        System.out.println((objectHeight - cameraHeight) / (Math.tan(0.367722982582348 + ty)));
        return (objectHeight - cameraHeight) / (Math.tan(0.367722982582348 + ty));
      }

     /* Determine the mounting angle of the camera given a vision target and its known distance, height off of the ground,
   and the height of the camera off of the ground. */
    public void determineMountingAngle(double distance, double cameraHeight, double objectHeight) {
        // NOTE: ty may be negative.
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        mounting_angle = Math.atan((cameraHeight - objectHeight) / distance) - ty;
        System.out.println(mounting_angle);
      }
    
}
