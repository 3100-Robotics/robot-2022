package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

public class Shooter extends SubsystemBase {

  private double tv, tx, ty, ta;
  private double mounting_angle;
  private double cameraHeight;
  private double hubHeight;
  public static WPI_TalonFX shooter = new WPI_TalonFX(MotorConstants.shooterMotorPort);
  private PeriodicIO mPeriodicIO;

  //TUNE P AND D FOR SHOOTER
  public Shooter() {

    mPeriodicIO = new PeriodicIO();

    /*A simple strategy for setting up a closed loop is to zero out all Closed-Loop Control Parameters 
    and start with the Feed-Forward Gain. Tune this until the sensed value is close to the target under typical load. 
    Then start increasing P gain so that the closed-loop will make up for the remaining error.

    “Ramping” can be configured using configClosedloopRamp (routine or VI)

    PIDF controller takes in target and sensor velocity measurements in “raw” sensor units per 100ms.
    */
    shooter.setNeutralMode(NeutralMode.Coast);
    shooter.configClosedloopRamp(0.5, Constants.kCANTimeoutMs);
    shooter.setSensorPhase(false);
    shooter.config_kP(0, 0);
    //shooter.config_kI(0, 0);
    shooter.config_kD(0, 0);
    shooter.config_kF(0, 0);
    //shooter.config_IntegralZone(0, 0, 50);

    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    shooter.selectProfileSlot(0, 0);

  }

  public void periodic() {

    mPeriodicIO.velocity_in_ticks_per_100ms = shooter.getSelectedSensorVelocity(0);
    mPeriodicIO.output_percent = shooter.getMotorOutputPercent();

  }

  public void shoot() {

    shooter.set(TalonFXControlMode.Velocity, computeSpeed(mounting_angle, cameraHeight, hubHeight) *  Constants.kSensorUnitsPerRotation / 600 / Constants.shooterGearRatio);

  }

  public void setVelocitySpeed(double RPM) {

    double _RPM = limit(RPM, 0, Constants.MAX_RPM);

    shooter.set(
        TalonFXControlMode.Velocity,
        (_RPM * Constants.kSensorUnitsPerRotation / 600 / Constants.shooterGearRatio));
  }

  public void stopShoot() {

    shooter.set(0);

  }

  // For the shooter. Given what the limelight sees and the shooter angle, compute
  // the desired initial speed for the shooter.
  public double computeSpeed(double angle, double cameraHeight, double objectHeight) {
    double distance = determineObjectDist(cameraHeight, objectHeight);
    return Math.sqrt((16.1 * Math.pow(distance, 2)) / (distance * Math.tan(angle) - cameraHeight - objectHeight))
        / Math.cos(angle);
  }

  /*
   * Determine the distance an object is from the limelight given the camera's
   * height
   * off of the ground and the object's height off of the ground.
   */
  public double determineObjectDist(double cameraHeight, double objectHeight) {
    System.out.println("Distance: ");
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    // 0.36 is mounting angle
    System.out.println((objectHeight - cameraHeight) / (Math.tan(0.367722982582348 + ty)));
    return (objectHeight - cameraHeight) / (Math.tan(0.367722982582348 + ty));
  }

  /*
   * Determine the mounting angle of the camera given a vision target and its
   * known distance, height off of the ground,
   * and the height of the camera off of the ground.
   */
  public void determineMountingAngle(double distance, double cameraHeight, double objectHeight) {
    // NOTE: ty may be negative.
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    mounting_angle = Math.atan((cameraHeight - objectHeight) / distance) - ty;
    System.out.println(mounting_angle);
  }

  private static class PeriodicIO {
    // inputs
    public double velocity_in_ticks_per_100ms;
    public double output_percent;

    // outputs
    public double setpoint_rpm;
  }

  public static double limit(double value, double limitValLow, double limitValHigh) {
    if (value > limitValHigh) {
      return limitValHigh;
    }
    if (value < limitValLow) {
      return limitValLow;
    }
    return value;
  }

}
