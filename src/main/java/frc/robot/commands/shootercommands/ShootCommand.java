package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends SequentialCommandGroup {
  public ShootCommand(Shooter m_shooter, double shooterSetpoint) {
    addCommands(new SetShooterVelocity(m_shooter, shooterSetpoint),
        new WaitForShooterVelocity(m_shooter, shooterSetpoint, 0.0, 1.0), new FeedShooter());
  }
}
