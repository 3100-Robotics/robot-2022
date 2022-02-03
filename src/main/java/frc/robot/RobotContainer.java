// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TurretLimeTurn;
import frc.robot.commands.VelocityDrive;
import frc.robot.commands.ZoomZoomFast;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.RobotCommands.*;



import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final XboxController m_driveController = new XboxController(driveControllerPort);
 // private final XboxController m_techController = new XboxController(techControllerPort);

  public final JoystickButton turnNeg = new JoystickButton(m_driveController, backButtonChannel);
  public final JoystickButton turnPos = new JoystickButton(m_driveController, startButtonChannel); 
  public final JoystickButton velocityDrive = new JoystickButton(m_driveController, rightTriggerChannel);
  public final JoystickButton limeTurn = new JoystickButton(m_driveController, xButtonChannel);
  public final JoystickButton determineDistance = new JoystickButton(m_driveController, aButtonChannel);
  public final JoystickButton findAngle = new JoystickButton(m_driveController, bButtonChannel);

  // public final JoystickButton m_deployCollector = new JoystickButton(m_techController, aButtonChannel);
  // public final JoystickButton m_groundCollect = new JoystickButton(m_techController, bButtonChannel);
  // public final JoystickButton m_conveyorRun = new JoystickButton(m_techController, xButtonChannel);
  // public final JoystickButton m_collectToShoot = new JoystickButton(m_techController, yButtonChannel);

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    new RobotCommands();

    m_drive.setDefaultCommand(
      // // A split-stick arcade command, with forward/backward controlled by the left
      // // hand, and turning controlled by the right.
      new ZoomZoomFast(m_drive, () -> m_driveController.getRightX(),
          () -> -m_driveController.getLeftY()));
  }

  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    turnPos.whileHeld(turnTurretPos);
    turnNeg.whileHeld(turnTurretNeg);
    limeTurn.whileHeld(new TurretLimeTurn(m_turret), true);
    velocityDrive.whileActiveContinuous(new VelocityDrive(m_drive, () -> -m_driveController.getLeftY(),
        () -> m_driveController.getRightX()));
    // m_deployCollector.whenPressed(deployCollectorCommand);
    // m_groundCollect.whileHeld(groundCollect);
    // m_conveyorRun.whileHeld(conveyorRun);
    // m_collectToShoot.whileHeld(collectToShoot);
    determineDistance.whenPressed(findDistance);
    findAngle.whenPressed(findMountAngle);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
