package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterMode;

/**
 * AutoAdjustHood - A command that toggles between set shooting positions
 * @see Shooter subsytem
*/

public class ToggleShooterMode extends InstantCommand {
  Shooter shooter;

  public ToggleShooterMode(Shooter shooterArg) {
    shooter = shooterArg;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(shooter.getShootMode() == ShooterMode.TARMAC){

      System.out.println("Launch");
      shooter.setShootMode(ShooterMode.LAUNCH);

    }else if (shooter.getShootMode() == ShooterMode.LAUNCH){

      System.out.println("Tarmac");
      shooter.setShootMode(ShooterMode.TARMAC);

    }

  }
}