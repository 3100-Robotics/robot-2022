// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;

public class Drive extends SubsystemBase {
  

  /** Config Objects for motor controllers */
  TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

  TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise;

  public static WPI_TalonFX frontLeft = new WPI_TalonFX(MotorConstants.frontLeftDrivePort);
  WPI_TalonFX backLeft = new WPI_TalonFX(MotorConstants.backLeftDrivePort);
  public static WPI_TalonFX frontRight = new WPI_TalonFX(MotorConstants.frontRightDrivePort);
  WPI_TalonFX backRight = new WPI_TalonFX(MotorConstants.backRightDrivePort);

  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
       backLeft,
      frontLeft);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
       backRight,
      frontRight);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public Drive() {
    setupDrivetrainMotors();
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    double leftDist = getLeftPosition();
    double rightDist = getRightPosition();
    double leftVel = getLeftVelocity();
    double rightVel = getRightVelocity();

    SmartDashboard.putNumber("LeftDistance", leftDist);
    SmartDashboard.putNumber("RightDistance", rightDist);
    SmartDashboard.putNumber("LeftVelocity", leftVel);
    SmartDashboard.putNumber("RightVelocity", rightVel);

    m_odometry.update(
        m_gyro.getRotation2d(),
        leftDist,
        rightDist);

    // System.out.println("Right Vel " + rightVel);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftVelocity(),
        getRightVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param forward the commanded forward movement
   * @param turn    the commanded rotation
   */
  public void arcadeDrive(double forward, double turn) {

    forward = Deadband(forward);
    turn = Deadband(turn);

     forward *= forward * forward;
     turn *= turn * turn;

    // frontLeft.set(TalonFXControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
    // frontRight.set(TalonFXControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
     m_drive.arcadeDrive(forward, turn);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace){
    xSpeed = Math.copySign(xSpeed*xSpeed, xSpeed);//*0.5;
    zRotation = Math.copySign(zRotation*zRotation, zRotation);
    m_drive.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeft.setVoltage(leftVolts);
    frontRight.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontRight.getSensorCollection().setIntegratedSensorPosition(0, 10);
    frontLeft.getSensorCollection().setIntegratedSensorPosition(0, 10);
  }

  double getLeftPosition() {
    // Native units are encoder ticks (2048 ticks per revolution)
    return // -1 *
    frontLeft.getSelectedSensorPosition() * DriveConstants.encoderScale;
    // / DriveConstants.kMagMultiplier;
  }

  double getRightPosition() {
    // Native units are encoder ticks (2048 ticks per revolution)
    return // -1 *
    frontRight.getSelectedSensorPosition() * DriveConstants.encoderScale;
  }

  double getLeftVelocity() {
    // Native units are encoder ticks per 100ms
    return // -1 *
    frontLeft.getSelectedSensorVelocity() * DriveConstants.encoderScale;
  }

  double getRightVelocity() {
    // Native units are encoder ticks per 100ms
    return // -1 *
    frontRight.getSelectedSensorVelocity() * DriveConstants.encoderScale;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
    /**
     * Determine if we need a Sum or Difference.
     * 
     * The auxiliary Talon FX will always be positive
     * in the forward direction because it's a selected sensor
     * over the CAN bus.
     * 
     * The master's native integrated sensor may not always be positive when forward
     * because
     * sensor phase is only applied to *Selected Sensors*, not native
     * sensor sources. And we need the native to be combined with the
     * aux (other side's) distance into a single robot distance.
     */

    /*
     * THIS FUNCTION should not need to be modified.
     * This setup will work regardless of whether the master
     * is on the Right or Left side since it only deals with
     * distance magnitude.
     */

    /* Check if we're inverted */
    if (masterInvertType == TalonFXInvertType.Clockwise) {
      /*
       * If master is inverted, that means the integrated sensor
       * will be negative in the forward direction.
       * 
       * If master is inverted, the final sum/diff result will also be inverted.
       * This is how Talon FX corrects the sensor phase when inverting
       * the motor direction. This inversion applies to the *Selected Sensor*,
       * not the native value.
       * 
       * Will a sensor sum or difference give us a positive total magnitude?
       * 
       * Remember the Master is one side of your drivetrain distance and
       * Auxiliary is the other side's distance.
       * 
       * Phase | Term 0 | Term 1 | Result
       * Sum: -((-)Master + (+)Aux )| NOT OK, will cancel each other out
       * Diff: -((-)Master - (+)Aux )| OK - This is what we want, magnitude will be
       * correct and positive.
       * Diff: -((+)Aux - (-)Master)| NOT OK, magnitude will be correct but negative
       */

      masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local Integrated Sensor
      masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
      masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); // Diff0
                                                                                                                  // -
                                                                                                                  // Diff1
    } else {
      /* Master is not inverted, both sides are positive so we can sum them. */
      masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
      masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local IntegratedSensor
      masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); // Sum0 +
                                                                                                           // Sum1
    }

    /*
     * Since the Distance is the sum of the two sides, divide by 2 so the total
     * isn't double
     * the real-world value
     */
    masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
  }

  double Deadband(double value) {
    /* Upper deadband */
    if (value >= +0.05)
      return value;

    /* Lower deadband */
    if (value <= -0.05)
      return value;

    /* Outside deadband */
    return 0;
  }

  public void setupDrivetrainMotors() {


    TalonFXConfiguration configs = new TalonFXConfiguration();

    frontLeft.configFactoryDefault();
    backLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backRight.configFactoryDefault();

    frontLeft.setSafetyEnabled(false);
    backLeft.setSafetyEnabled(false);
    frontRight.setSafetyEnabled(false);
    backRight.setSafetyEnabled(false);
    

    // _leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    // _rightConfig.remoteFilter0.remoteSensorDeviceID = frontLeft.getDeviceID(); // Device ID of Remote Source
    // _rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Source Type

    // setRobotDistanceConfigs(_rightInvert, _rightConfig);
    // configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    // frontLeft.configAllSettings(configs);
    // frontRight.configAllSettings(configs);

    // Determines which motors will be inverted

    frontLeft.setInverted(false);
    frontLeft.setInverted(false);
    frontRight.setInverted(true);
    backRight.setInverted(true);

    // Sets the motors to brake mode
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    frontLeft.configPeakOutputForward(1.0);
    frontLeft.configPeakOutputReverse(-1.0);

    frontRight.configPeakOutputForward(1.0);
    frontRight.configPeakOutputReverse(-1.0);

    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // _rightConfig.slot2.kF = Constants.kGains_Velocit.kF;
    // _rightConfig.slot2.kP = Constants.kGains_Velocit.kP;
    // _rightConfig.slot2.kI = Constants.kGains_Velocit.kI;
    // _rightConfig.slot2.kD = Constants.kGains_Velocit.kD;
    // _rightConfig.slot2.integralZone = Constants.kGains_Velocit.kIzone;
    // _rightConfig.slot2.closedLoopPeakOutput = Constants.kGains_Velocit.kPeakOutput;

    // /* Config the neutral deadband. */
    // _leftConfig.neutralDeadband = Constants.kNeutralDeadband;
    // _rightConfig.neutralDeadband = Constants.kNeutralDeadband;

    // int closedLoopTimeMs = 1;
    // _rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    // _rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    // _rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    // _rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

    // /* Motion Magic Configs */
    // _rightConfig.motionAcceleration = 2000; // (distance units per 100 ms) per second
    // _rightConfig.motionCruiseVelocity = 2000; // distance units per 100 ms

    // /* APPLY the config settings */
    // frontLeft.configAllSettings(_leftConfig);
    // frontRight.configAllSettings(_rightConfig);

    // /* Set status frame periods to ensure we don't have stale data */
    // frontRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    // frontRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    // frontLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);


  

    //leftMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentThreshold, currentThresholdTime));
    //rightMotorLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentThreshold, currentThresholdTime));

    //Might break
    frontLeft.setSensorPhase(true);
    frontRight.setSensorPhase(true);

    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);

    // Add PID constants
    frontLeft.config_kP(0, 0);
    frontLeft.config_kI(0, 0);
    frontLeft.config_kD(0, 0);
    frontLeft.config_kF(0, 0);
    // leftMotorLeader.configMaxIntegralAccumulator(0, 400);

    frontRight.config_kP(0, 0);
    frontRight.config_kI(0, 0);
    frontRight.config_kD(0, 0);
    frontRight.config_kF(0, 0);
    // rightMotorLeader.configMaxIntegralAccumulator(0, 400);
 
    frontLeft.setIntegralAccumulator(0);
    frontRight.setIntegralAccumulator(0);
  }

}
