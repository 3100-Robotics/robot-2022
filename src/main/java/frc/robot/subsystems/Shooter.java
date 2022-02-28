package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sensors.LimelightInterface;
import frc.robot.subsystems.Turret;

public class Shooter extends SubsystemBase {

 
  private double mounting_angle;
  //TODO: Find
  private double maxVelocity = 21666;
  private double cameraHeight = 0.6096;
  private double hubHeight = 2.6416;
  private boolean isUpToSpeed = false;
  private double actual_speed;
  private double setSpeed;

  public enum ShooterMode {
    TARMAC,
    LAUNCH
  }

  private ShooterMode shooterMode = ShooterMode.TARMAC;
  private double shooterSpeed; //= ShooterConstants.TARMAC_SHOOT_SPEED;

  public static WPI_TalonFX shooter = new WPI_TalonFX(MotorConstants.shooterMotorPort);

  //TUNE P AND D FOR SHOOTER
  public Shooter() {

    

    /*A simple strategy for setting up a closed loop is to zero out all Closed-Loop Control Parameters 
    and start with the Feed-Forward Gain. Tune this until the sensed value is close to the target under typical load. 
    Then start increasing P gain so that the closed-loop will make up for the remaining error.

    Ramping can be configured using configClosedloopRamp (routine or VI)

    PIDF controller takes in target and sensor velocity measurements in raw sensor units per 100ms.
    */

    shooter.configFactoryDefault();
    shooter.setNeutralMode(NeutralMode.Coast);
    //shooter.configClosedloopRamp(0.5, Constants.kCANTimeoutMs);
    // shooter.setSensorPhase(false);
     shooter.config_kP(0, 0.53709);
    // //shooter.config_kI(0, 0);
    // //shooter.config_kD(0, 0);
    // shooter.config_kF(0, 0);
    //shooter.config_IntegralZone(0, 0, 50);

    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooter.setInverted(true);
    //shooter.configClosedloopRamp(0.25);
    //shooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

   // shooter.selectProfileSlot(0, 0);

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
        Turret.adjustHoodAuton(30);
        shooterSpeed = ShooterConstants.TARMAC_SHOOT_SPEED;
        break;
      case LAUNCH:
        Turret.adjustHoodAuton(110);
        shooterSpeed = ShooterConstants.LAUNCH_SHOOT_SPEED;
        break;
      default:
        Turret.adjustHoodAuton(30);
        shooterSpeed = ShooterConstants.TARMAC_SHOOT_SPEED;
        break;
    }
  }

  public void shootPercent() {
    shooter.set(TalonFXControlMode.PercentOutput, shooterSpeed);
  }

  public void shoot() {
    shooter.set(TalonFXControlMode.Velocity, LimelightInterface.computeSpeed(mounting_angle, cameraHeight, hubHeight) * ShooterConstants.shooterSensorToRealDistanceFactor); //Constants.kSensorUnitsPerRotation / 600 / Constants.shooterGearRatio);
  }

  public void stopShoot() {
    shooter.set(0);
  }

 

  public double getRPM() {
		return (shooter.getSelectedSensorVelocity() * 10 * 60 / (2048 * (2/3)));
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

  public void periodic() {
    setShooter();
  }



  public boolean isPrimed() {
	
		return Math.abs(setSpeed + getRPM()) <= ShooterConstants.SHOOTER_TOLERANCE;
	}





//NOT WORKING THINGS


// public double getVelocity() {
//   return shooter.getSelectedSensorVelocity() / maxVelocity;

// }
// public void setPercentVelocity(double percentVelocity) {
//   System.out.println("AHHHHH");
//   shooter.set(TalonFXControlMode.Velocity, maxVelocity * percentVelocity);
// }


// public void setVelocitySpeed(double RPM) {

//   System.out.println("SHOOTING");
//   double _RPM = limit(RPM, 0, ShooterConstants.MAX_RPM);

//   shooter.set(
//       TalonFXControlMode.Velocity,
//       (_RPM * ShooterConstants.shooterSensorToRealDistanceFactor));//Constants.kSensorUnitsPerRotation / 600 / Constants.shooterGearRatio));
// }








}
