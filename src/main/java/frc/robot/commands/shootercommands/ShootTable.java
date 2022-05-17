package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sensors.LimelightInterface;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShootTable extends CommandBase {
  private Shooter m_shooter;
  private Turret m_turret;
  ///private Timer timer = new Timer();

  public ShootTable(Shooter m_subsystem, Turret m_turret) {
    m_shooter = m_subsystem;
    
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    // System.out.println(LimelightInterface.getDistance());
    // System.out.println(LimelightInterface.getDistance());
    // System.out.println(LimelightInterface.getDistance());
    // System.out.println(LimelightInterface.getDistance());
    // System.out.println(LimelightInterface.getDistance());
    // timer.reset();
    // timer.start();
  }

  @Override
  public void execute() {


    
    m_shooter.setRPM(2000);
   // m_shooter.setRPM(ShooterConstants.DIST_TO_RPM_TABLE.get(LimelightInterface.getDistance()));
   //Turret.adjustHoodAuton(ShooterConstants.DIST_TO_HOOD_ANGLE_TABLE.get(LimelightInterface.getDistance()));

  }

  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putNumber("Last Spinup Time", timer.get());
    // timer.stop();
  }

  // @Override
  // public boolean isFinished() {
  //  // return true;
  // //   boolean hasAchievedVelocity = m_shooter.isAtSetpoint();
  // //   return hasAchievedVelocity;
  // //   // boolean hasTimeElapsed = timer.get() >= minWaitSeconds;
  // //   // boolean cantWaitAnymore = timer.get() >= maxWaitSeconds;
  // //   //return (hasTimeElapsed && hasAchievedVelocity) || cantWaitAnymore;
  // }
}