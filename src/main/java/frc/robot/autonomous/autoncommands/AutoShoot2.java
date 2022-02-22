/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.autoncommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoShoot2 extends CommandBase {
  private long startTime;
  private long timeOutMs;

  public int time = 0;
  public static double INDEXER_RUN_SPEED = 0.3;
  public static double SINGULATOR_RUN_SPEED = 0.5;

  public static double MANUAL_RUN_SPEED_SHOOTER = 0.5;
  public static double TARGET_RUN_SPEED_SHOOTER = 2100; // ideal speed in RPM
  public static double RUN_TOLERANCE_SHOOTER = 50; // tolerance range for shooter speed
  public static double MAINTAIN_RUN_SPEED_SHOOTER = 0.35; // want this to roughly hold target RPM
  public static double SLOW_RUN_SPEED_SHOOTER = MAINTAIN_RUN_SPEED_SHOOTER - 0.04; // want this to slow down a bit but not fully
  public static double MANUAL_REDUCTION = 0.2;
  public static double MIN_RUN_SPEED = 0.05;
  private double actual_speed = 0;
  private double current_speed = 0;



  public AutoShoot2(long timeOutMs) {
    this.timeOutMs = timeOutMs;
    // Use requires() here to declare subsystem dependencies
  }

  // Called just before this Command runs the first time
  @Override
public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
public void execute() {
    if ((System.currentTimeMillis() - startTime) < (timeOutMs - 2000)) {
      actual_speed = Shooter.shooter.getSelectedSensorVelocity() * ShooterConstants.shooterSensorToRealDistanceFactor;//Constants.shooterEncoderRatio;
      if (actual_speed < TARGET_RUN_SPEED_SHOOTER - RUN_TOLERANCE_SHOOTER) {
        current_speed = 1.0; // speed up as quickly as possible
      } else if (actual_speed < TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER) {
        current_speed = MAINTAIN_RUN_SPEED_SHOOTER;
      } else {
        // implies actual_speed >= TARGET_RUN_SPEED_SHOOTER + RUN_TOLERANCE_SHOOTER
        current_speed = SLOW_RUN_SPEED_SHOOTER;
      }
     // Shooter.setVelocitySpeed(current_speed);

    }
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
public boolean isFinished() {
    return ((startTime + timeOutMs) < System.currentTimeMillis() );
  }
}