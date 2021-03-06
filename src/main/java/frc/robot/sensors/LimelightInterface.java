/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.util.Util;

public class LimelightInterface extends SubsystemBase {

  // Limelight docs say to use at least 11ms
  private static final double imageCaptureLatency = 11; // ms

  private static double ty;
  private static NetworkTableEntry targetValid;
  private NetworkTableEntry horizAngle;
  private static NetworkTableEntry vertAngle;
  private NetworkTableEntry area;
  private NetworkTableEntry skew;
  private NetworkTableEntry pipelineLatency;
  private NetworkTableEntry shortSideLength;
  private NetworkTableEntry longSideLength;
  private NetworkTableEntry horizSideLength;
  private NetworkTableEntry vertSideLength;
  private NetworkTableEntry pipelineGet;
  private NetworkTableEntry pipelineSet;
  private NetworkTableEntry camtran;
  private static NetworkTableEntry ledMode;
  private NetworkTableEntry camMode;
  private NetworkTableEntry streamingMode;
  private NetworkTableEntry snapshot;
  private NetworkTableEntry contourXs;
  private NetworkTableEntry contourYs;
  private NetworkTableEntry rawContour0X;
  private NetworkTableEntry rawContour0Y;
  private NetworkTableEntry rawContour0Area;
  private NetworkTableEntry rawContour0Skew;
  private NetworkTableEntry rawContour1X;
  private NetworkTableEntry rawContour1Y;
  private NetworkTableEntry rawContour1Area;
  private NetworkTableEntry rawContour1Skew;
  private NetworkTableEntry rawContour2X;
  private NetworkTableEntry rawContour2Y;
  private NetworkTableEntry rawContour2Area;
  private NetworkTableEntry rawContour2Skew;
  private NetworkTableEntry crosshairAX;
  private NetworkTableEntry crosshairAY;
  private NetworkTableEntry crosshairBX;
  private NetworkTableEntry crosshairBY;

  /**
   * Creates a new LimelightInterface.
   */
  public LimelightInterface() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    targetValid = table.getEntry("tv");
    horizAngle = table.getEntry("tx");
    vertAngle = table.getEntry("ty");
    area = table.getEntry("ta");
    skew = table.getEntry("ts");
    pipelineLatency = table.getEntry("tl");
    shortSideLength = table.getEntry("tshort");
    longSideLength = table.getEntry("longv");
    horizSideLength = table.getEntry("thor");
    vertSideLength = table.getEntry("tvert");
    pipelineGet = table.getEntry("getpipe");
    pipelineSet = table.getEntry("pipeline");
    camtran = table.getEntry("camtran");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
    streamingMode = table.getEntry("stream");
    snapshot = table.getEntry("snapshot");
    contourXs = table.getEntry("tcornx");
    contourYs = table.getEntry("tcorny");
    rawContour0X = table.getEntry("tx0");
    rawContour0Y = table.getEntry("ty0");
    rawContour0Area = table.getEntry("ta0");
    rawContour0Skew = table.getEntry("ts0");
    rawContour1X = table.getEntry("tx1");
    rawContour1Y = table.getEntry("ty1");
    rawContour1Area = table.getEntry("ta1");
    rawContour1Skew = table.getEntry("ts1");
    rawContour2X = table.getEntry("tx2");
    rawContour2Y = table.getEntry("ty2");
    rawContour2Area = table.getEntry("ta2");
    rawContour2Skew = table.getEntry("ts2");
    crosshairAX = table.getEntry("cx0");
    crosshairAY = table.getEntry("cy0");
    crosshairBX = table.getEntry("cx1");
    crosshairBY = table.getEntry("cy1");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the LED mode of the limelight.
   * 
   * @param mode The LED mode
   */
  public static void setLEDMode(LimelightLEDMode mode) {
    ledMode.forceSetNumber(mode.modeID);
  }

  public void useAsDriverCam(boolean driverCam) {
    camMode.setDouble(driverCam ? 1.0 : 0.0);
  }

  /**
   * Gets the current pipeline number.
   * 
   * @return the pipeline number, 0 to 9
   */
  public int getCurrentPipeline() {
    return pipelineGet.getNumber(0).intValue();
  }

  /**
   * Sets the current pipeline.
   * 
   * @param pipelineNumber the pipeline number, 0 to 9
   */
  public void setPipeline(int pipelineNumber) {
    pipelineSet.forceSetNumber(pipelineNumber);
  }

  /**
   * Returns whether the limelight currently sees a valid target,
   * 
   * @return whether there is a valid target
   */
  public static boolean hasValidTarget() {
    return targetValid.getDouble(0) != 0.0;
  }

  /**
   * Gets the estimated latency between image capture and the current data.
   * 
   * @return latency in milliseconds
   */
  public double getLatency() {
    return pipelineLatency.getDouble(0) + imageCaptureLatency;
  }

  /**
   * Sets the streaming mode of the limelight.
   * 
   * @param mode The streaming mode
   */
  public void setStreamingMode(LimelightStreamingMode mode) {
    streamingMode.forceSetNumber(mode.modeID);
  }

  /**
   * Enables or disables snapshots. If enabled, two snapshots are captured per
   * second.
   * 
   * @param enable whether to enable snapshots
   */
  public void enableSnapshots(boolean enable) {
    snapshot.setDouble(enable ? 1.0 : 0.0);
  }

  /**
   * Gets the horizontal angle to the target.
   * 
   * @return the horizontal angle, +/-29.8 deg
   */
  public double getTargetHorizAngle() {
    return horizAngle.getDouble(0);
  }

  /**
   * Gets the vertical angle to the target.
   * 
   * @return the vertical angle, +/-24.85 deg
   */
  public static double getTargetVertAngle() {
    return vertAngle.getDouble(0);
  }

  /**
   * Gets the area of the target.
   * 
   * @return the area, 0 to 100%
   */
  public double getTargetArea() {
    return area.getDouble(0);
  }

  /**
   * Gets the target's skew/rotation.
   * 
   * @return the target's skew, -90 to 0 degrees
   */
  public double getTargetSkew() {
    return skew.getDouble(0);
  }

  /**
   * Gets the length of the shortest side of the fitted bounding box around the
   * target.
   * 
   * @return the shortest side length in pixels
   */
  public double getTargetshortSideLength() {
    return shortSideLength.getDouble(0);
  }

  /**
   * Gets the length of the longest side of the fitted bounding box around the
   * target.
   * 
   * @return the longest side length in pixels
   */
  public double getTargetLongSideLength() {
    return longSideLength.getDouble(0);
  }

  /**
   * Gets the horizontal side length of the fitted bounding box around the target.
   * 
   * @return the the horizontal length, 0 to 320 pixels at standard resolution
   */
  public double getTargetHorizSideLength() {
    return horizSideLength.getDouble(0);
  }

  /**
   * Gets the vertical side length of the fitted bounding box around the target.
   * 
   * @return the the vertical length, 0 to 320 pixels at standard resolution
   */
  public double getTargetVertSideLength() {
    return vertSideLength.getDouble(0);
  }

  /**
   * Gets the 3D position solution.
   * 
   * @return a 6-length array (x, y, z, pitch, yaw, roll)
   */
  public double[] get3DSolution() {
    return camtran.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
  }

  /**
   * Gets the x values of the contour of the target. "Send contours" must be
   * enabled in the Output tab on the Limelight
   * 
   * @return an array of x values
   */
  public double[] getContourXs() {
    return contourXs.getDoubleArray(new double[] {});
  }

  /**
   * Gets the y values of the contour of the target. "Send contours" must be
   * enabled in the Output tab on the Limelight
   * 
   * @return an array of y values
   */
  public double[] getContourYs() {
    return contourYs.getDoubleArray(new double[] {});
  }

  /**
   * Gets the x value of a crosshair.
   * 
   * @param crosshair which crosshair
   * @return the x value in normalized screen space
   */
  public double getCrosshairX(LimelightCrosshair crosshair) {
    NetworkTableEntry entry;
    switch (crosshair) {
      default:
      case A:
        entry = crosshairAX;
        break;
      case B:
        entry = crosshairBX;
        break;
    }
    return entry.getDouble(0);
  }

  /**
   * Gets the y value of a crosshair.
   * 
   * @param crosshair which crosshair
   * @return the y value in normalized screen space
   */
  public double getCrosshairY(LimelightCrosshair crosshair) {
    NetworkTableEntry entry;
    switch (crosshair) {
      default:
      case A:
        entry = crosshairAY;
        break;
      case B:
        entry = crosshairBY;
        break;
    }
    return entry.getDouble(0);
  }

  /**
   * Gets the x value of a raw contour.
   * 
   * @param contourID which contour, 0 to 2
   * @return raw screenspace x, -1 to 1
   */
  public double getRawContourX(int contourID) {
    NetworkTableEntry entry;
    switch (contourID) {
      default:
      case 0:
        entry = rawContour0X;
        break;
      case 1:
        entry = rawContour1X;
        break;
      case 2:
        entry = rawContour2X;
        break;
    }
    return entry.getDouble(0);
  }

  /**
   * Gets the y value of a raw contour.
   * 
   * @param contourID which contour, 0 to 2
   * @return raw screenspace y, -1 to 1
   */
  public double getRawContourY(int contourID) {
    NetworkTableEntry entry;
    switch (contourID) {
      default:
      case 0:
        entry = rawContour0Y;
        break;
      case 1:
        entry = rawContour1Y;
        break;
      case 2:
        entry = rawContour2Y;
        break;
    }
    return entry.getDouble(0);
  }

  /**
   * Gets the area of a raw contour.
   * 
   * @param contourID which contour, 0 to 2
   * @return area, 0 to 100%
   */
  public double getRawContourArea(int contourID) {
    NetworkTableEntry entry;
    switch (contourID) {
      default:
      case 0:
        entry = rawContour0Area;
        break;
      case 1:
        entry = rawContour1Area;
        break;
      case 2:
        entry = rawContour2Area;
        break;
    }
    return entry.getDouble(0);
  }

  /**
   * Gets the skew of a raw contour.
   * 
   * @param contourID which contour, 0 to 2
   * @return skew, -90 to 0 degrees
   */
  public double getRawContourSkew(int contourID) {
    NetworkTableEntry entry;
    switch (contourID) {
      default:
      case 0:
        entry = rawContour0Skew;
        break;
      case 1:
        entry = rawContour1Skew;
        break;
      case 2:
        entry = rawContour2Skew;
        break;
    }
    return entry.getDouble(0);
  }

  public enum LimelightLEDMode {
    PIPELINE(0), OFF(1), BLINK(2), ON(3);

    public final int modeID;

    private LimelightLEDMode(int modeID) {
      this.modeID = modeID;
    }
  }

  public enum LimelightStreamingMode {
    /**
     * Dual side-by-side streams
     */
    STANDARD(0),
    /**
     * Secondary camera stream in the lower-right of the primary stream
     */
    PIP_MAIN(1),
    /**
     * Primary camera stream in the lower-right of the secondary stream
     */
    PIP_SECONDARY(2);

    public final int modeID;

    private LimelightStreamingMode(int modeID) {
      this.modeID = modeID;
    }
  }

  public enum LimelightCrosshair {
    A, B;
  }

  /**
   * Calculates the distance to the target[hoop] based on the Y of the target
   * given by the limelight.
   * Does so by taking the value the limelight gives from it's 'ty' network table
   * entry
   * (table for sending arbitrary values between the limelight, robot, and driver
   * station)
   * and putting it through the trigonometrical coding and algorithms
   *
   * @return The distance to the target(hoop) in inches
   */
  public static double getDistance() {
    return Util.findDist(LimelightConstants.LIMELIGHT_HEIGHT, LimelightConstants.LIMELIGHT_ELEVATION,
        LimelightConstants.BASKET_HEIGHT, getTargetVertAngle());
  }

  public static double getRobotToTargetDistance() {
    return (LimelightConstants.LL_TARGET_HEIGHT - LimelightConstants.LL_MOUNT_HEIGHT)
        / Math.tan(Math.toRadians(LimelightConstants.LL_MOUNT_ANGLE + getTargetVertAngle()));
  }

  // public double getShooterLaunchVelocity() {
  // double g = 9.81;
  // double y = LimelightConstants.LL_TARGET_HEIGHT;
  // double x = getRobotToTargetDistance();
  // double launchAngle = LimelightConstants.SHOOTER_LAUNCH_ANGLE; // Set to
  // proper value
  // double tanA = Math.tan(Math.toRadians(launchAngle));
  // double upper = Math.sqrt(g) * Math.sqrt(x) * Math.sqrt(Math.pow(tanA, 2) +
  // 1);
  // double lower = Math.sqrt(2 * tanA - ((2 * y) / x));
  // return Units.metersToFeet(upper / lower);
  // }

  // For the shooter. Given what the limelight sees and the shooter angle, compute
  // the desired initial speed for the shooter.
  public static double computeSpeed(double angle, double cameraHeight, double objectHeight) {
    double distance = determineObjectDist(cameraHeight, objectHeight);
    return Math.sqrt((16.1 * Math.pow(distance, 2)) / (distance * Math.tan(angle) - cameraHeight - objectHeight))
        / Math.cos(angle);
  }

  /*
   * Determine the distance an object is from the limelight given the camera's
   * height
   * off of the ground and the object's height off of the ground.
   */
  public static double determineObjectDist(double cameraHeight, double objectHeight) {
    System.out.println("Distance: ");
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    // 0.36 is mounting angle
    System.out.println((objectHeight - cameraHeight) / (Math.tan(LimelightConstants.LL_MOUNT_ANGLE + ty)));
    return (objectHeight - cameraHeight) / (Math.tan(LimelightConstants.LL_MOUNT_ANGLE + ty));
  }

}
