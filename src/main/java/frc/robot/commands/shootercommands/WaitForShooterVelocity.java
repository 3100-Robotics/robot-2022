package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WaitForShooterVelocity extends CommandBase {
  private Shooter m_shooter;
  private Timer timer = new Timer();
 // private double minWaitSeconds = 0.0;
  private double rpm = 0.0;
 // private double maxWaitSeconds = 0.0;

  public WaitForShooterVelocity(Shooter m_subsystem, double rpm) {
    m_shooter = m_subsystem;
    addRequirements(m_shooter);
   // this.minWaitSeconds = minWaitSeconds;
    this.rpm = rpm;
  //  this.maxWaitSeconds = maxWaitSeconds;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {

  //  System.out.println();
    m_shooter.setRPM(rpm);

  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Last Spinup Time", timer.get());
    m_shooter.stopShoot();
    timer.stop();
  }

  // @Override
  // public boolean isFinished() {
  //   boolean hasAchievedVelocity = m_shooter.isAtSetpoint();
  //   return hasAchievedVelocity;
  //   // boolean hasTimeElapsed = timer.get() >= minWaitSeconds;
  //   // boolean cantWaitAnymore = timer.get() >= maxWaitSeconds;
  //   //return (hasTimeElapsed && hasAchievedVelocity) || cantWaitAnymore;
  // }
}