package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/**
 * Code should inline a command this simple with
 * {@link edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class VelocityDrive extends CommandBase {
  private final Drive m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward   The control input for driving forwards/backwards
   * @param rotation  The control input for turning
   */
  public VelocityDrive(Drive subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
    m_drive = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    addRequirements(m_drive);
  }

  public void initalize(){

    Drive.frontRight.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);

  }

  @Override
  public void execute() {

    double target_RPM = m_forward.getAsDouble() * 2000;	// +- 2000 RPM
	double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;	//RPM -> Native units
	double feedFwdTerm = m_rotation.getAsDouble() * 0.10;	// Percentage added to the close loop output
			
			/* Configured for Velocity Closed Loop on Integrated Sensors' Sum and Arbitrary FeedForward on joyX */
	Drive.frontRight.set(TalonFXControlMode.Velocity, target_unitsPer100ms, DemandType.ArbitraryFeedForward, feedFwdTerm);
	Drive.frontLeft.follow(Drive.frontRight);

  }
}