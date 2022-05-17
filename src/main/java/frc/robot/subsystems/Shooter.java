package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sensors.LimelightInterface;
import frc.robot.subsystems.Turret;
import frc.robot.util.Util;

public class Shooter extends SubsystemBase {

  private double mounting_angle;
  private double cameraHeight = 0.6096;
  private double hubHeight = 2.6416;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.84516, 0.12826, 0.013541);
  public static final double kF = (1023 / 0.75 * ShooterConstants.shooterSensorToRealDistanceFactor * 1023);
 //public static final double kF = 1023 * 0.75 / 2048;
  private double setSpeed;

  public enum ShooterMode {
    TARMAC, LAUNCH
  }

  public enum ShooterControlState {
    OPEN_LOOP, VELOCITY
  }

  private ShooterControlState mControlState = ShooterControlState.OPEN_LOOP;

  public static class PeriodicIO {
    // inputs
    public double timestamp;
    public double output_voltage;
    public double supply_current;
    public double stator_current;
    public double velocity_ticks_per_100_ms = 0.0;
    public double velocity_rpm = 0.0;

    // outputs
    public double demand = 0.0;
  }

  private ShooterMode shooterMode = ShooterMode.LAUNCH;
  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private double shooterSpeed; // = ShooterConstants.TARMAC_SHOOT_SPEED;

  public static WPI_TalonFX shooter = new WPI_TalonFX(MotorConstants.shooterMotorPort);

  // TUNE P AND D FOR SHOOTER
  public Shooter() {

    /*
     * A simple strategy for setting up a closed loop is to zero out all Closed-Loop
     * Control Parameters
     * and start with the Feed-Forward Gain. Tune this until the sensed value is
     * close to the target under typical load.
     * Then start increasing P gain so that the closed-loop will make up for the
     * remaining error.
     * 
     * Ramping can be configured using configClosedloopRamp (routine or VI)
     * 
     * PIDF controller takes in target and sensor velocity measurements in raw
     * sensor units per 100ms.
     */

    shooter.configFactoryDefault();
    shooter.setNeutralMode(NeutralMode.Coast);
    // shooter.configClosedloopRamp(0.5, Constants.kCANTimeoutMs);
    // shooter.setSensorPhase(false);
    //0.16739
   // shooter.config_kP(0, 0.16739);
    //shooter.config_kI(0, 0.5);
   // shooter.config_kD(0, 0);
    //0.65-7
   // shooter.config_kF(0, 0.0635);
    // 1023 * 0.5 / 
    // shooter.config_IntegralZone(0, 0, 50);

    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooter.setInverted(true);
    // shooter.configClosedloopRamp(0.25);
    // shooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    // shooter.selectProfileSlot(0, 0);

  }

  public void print(){

    System.out.println(LimelightInterface.getDistance());

  }

  public void readPeriodicInputs() {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp();

    mPeriodicIO.output_voltage = shooter.getMotorOutputVoltage();
    mPeriodicIO.supply_current = shooter.getSupplyCurrent();
    mPeriodicIO.stator_current = shooter.getStatorCurrent();
    mPeriodicIO.velocity_ticks_per_100_ms = shooter.getSelectedSensorVelocity(0);

    mPeriodicIO.velocity_rpm = nativeUnitsToRPM(mPeriodicIO.velocity_ticks_per_100_ms);

  }

  public synchronized void setOpenLoop(double power) {
    if (mControlState != ShooterControlState.OPEN_LOOP) {
      mControlState = ShooterControlState.OPEN_LOOP;
    }

    mPeriodicIO.demand = power;
  }

  public synchronized void setRPM(double rpm) {
    if (mControlState != ShooterControlState.VELOCITY) {
      mControlState = ShooterControlState.VELOCITY;
    }

   // mPeriodicIO.demand = rpmToNativeUnits(rpm);
   System.out.println("Velocity " + nativeUnitsToRPM(shooter.getSelectedSensorVelocity()));
   shooter.set(ControlMode.Velocity, rpmToNativeUnits(rpm));
  }

  public void writePeriodicOutputs() {
    if (mControlState == ShooterControlState.OPEN_LOOP) {
      shooter.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    } else if (mControlState == ShooterControlState.VELOCITY) {
      shooter.set(ControlMode.Velocity, mPeriodicIO.demand);
    }
  }

  public ShooterMode getShootMode() {
    return shooterMode;
  }

  public void setShootMode(ShooterMode mode) {
    shooterMode = mode;
  }

  public void setShooter() {
    switch (shooterMode) {
      case TARMAC:
        // System.out.println("Setting to Tarmac");
       // Turret.adjustHoodAuton(35);
        shooterSpeed = ShooterConstants.TARMAC_SHOOT_SPEED;
        SmartDashboard.putString("Shooter State", "Tarmac");
        break;
      case LAUNCH:
        // System.out.println("Setting to Launch");
       // Turret.adjustHoodAuton(0);
        shooterSpeed = ShooterConstants.LAUNCH_SHOOT_SPEED;
        SmartDashboard.putString("Shooter State", "Launch");
        break;
      default:
        System.out.println("Setting to Default");
       // Turret.adjustHoodAuton(35);
        shooterSpeed = ShooterConstants.TARMAC_SHOOT_SPEED;
        break;
    }
  }

  public void shootPercent() {
    System.out.println("Velocity " + nativeUnitsToRPM(shooter.getSelectedSensorVelocity()));
    shooter.set(TalonFXControlMode.PercentOutput, ShooterConstants.LAUNCH_SHOOT_SPEED);
  }

  public void shootPercentAuton(double speed) {
    shooter.set(TalonFXControlMode.PercentOutput, speed);
  }

  // public void shoot() {
  //   shooter.set(TalonFXControlMode.Velocity, LimelightInterface.computeSpeed(mounting_angle, cameraHeight, hubHeight)
  //       * ShooterConstants.shooterSensorToRealDistanceFactor); // Constants.kSensorUnitsPerRotation / 600 /
  //                                                              // Constants.shooterGearRatio);
  // }

  public void stopShoot() {
    shooter.set(0);
  }

  // public double getRPM() {
  //   return (shooter.getSelectedSensorVelocity() * 10 * 60 / (2048 * (2 / 3)));
  // }

  public static double limit(double value, double limitValLow, double limitValHigh) {
    if (value > limitValHigh) {
      return limitValHigh;
    }
    if (value < limitValLow) {
      return limitValLow;
    }
    return value;
  }

  public void periodic() {
    //System.out.println("Velocity " + nativeUnitsToRPM(shooter.getSelectedSensorVelocity()));
    //setShooter();
    //writePeriodicOutputs();
    SmartDashboard.putBoolean("Shooter At Setpoint", isAtSetpoint(4300));
    // SmartDashboard.putNumber("Shooter Demand", mControlState == ShooterControlState.OPEN_LOOP ? mPeriodicIO.demand
    //     : (mControlState == ShooterControlState.VELOCITY ? nativeUnitsToRPM(mPeriodicIO.demand) : 0.0));
  }

  /**
   * @param ticks per 100 ms
   * @return rpm
   */
  public double nativeUnitsToRPM(double ticks_per_100_ms) {
    return ticks_per_100_ms * 10.0 * 60.0 / Constants.kSensorUnitsPerRotation;
  }

  public double rpmToNativeUnits(double rpm) {
    return rpm / 60.0 / 10.0 * Constants.kSensorUnitsPerRotation;
  }

  public synchronized double getRPM() {
    return nativeUnitsToRPM(getVelocityNativeUnits());
  }

  public synchronized double getVelocityNativeUnits() {
    return shooter.getSelectedSensorVelocity();//mPeriodicIO.velocity_ticks_per_100_ms;
  }


  //B term should be the one that's found through running the 

  public synchronized boolean isAtSetpoint(double rpm) {
    return Util.epsilonEquals(getRPM(), rpm,
        Constants.kShooterAllowableErrorRPM);
  }

  

  // NOT WORKING THINGS

  // public double getVelocity() {
  // return shooter.getSelectedSensorVelocity() / maxVelocity;

  // }
  // public void setPercentVelocity(double percentVelocity) {
  // System.out.println("AHHHHH");
  // shooter.set(TalonFXControlMode.Velocity, maxVelocity * percentVelocity);
  // }

  // public void setVelocitySpeed(double RPM) {

  // System.out.println("SHOOTING");
  // double _RPM = limit(RPM, 0, ShooterConstants.MAX_RPM);

  // shooter.set(
  // TalonFXControlMode.Velocity,
  // (_RPM *
  // ShooterConstants.shooterSensorToRealDistanceFactor));//Constants.kSensorUnitsPerRotation
  // / 600 / Constants.shooterGearRatio));
  // }

}
