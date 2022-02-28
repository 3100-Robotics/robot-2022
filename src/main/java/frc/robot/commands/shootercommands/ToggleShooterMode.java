// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootercommands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterMode;

public class ToggleShooterMode extends InstantCommand {
  Shooter shooter;

  public ToggleShooterMode(Shooter shooterArg) {
    shooter = shooterArg;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (shooter.getShootMode()) {
      case TARMAC:
      System.out.println("Tarmac");
        shooter.setShootMode(ShooterMode.TARMAC);
        break;
      case LAUNCH:
      System.out.println("Launch");
        shooter.setShootMode(ShooterMode.LAUNCH);
        break;
      default:
        shooter.setShootMode(ShooterMode.TARMAC);
        break;
    }
  }
}