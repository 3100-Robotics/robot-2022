// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Drivetrain.CurvyDrive;
import frc.robot.Drivetrain.VelocityDrive;
import frc.robot.Drivetrain.ZoomZoomFast;
import frc.robot.autonomous.autoncommands.AutoAdjustHood;
import frc.robot.autonomous.autonroutes.TestGrouping;
import frc.robot.commands.Aim;
import frc.robot.commands.AutonAlign;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveLimeTurn;
import frc.robot.commands.LimeTurn;
import frc.robot.commands.TurretLimeTurn;
import frc.robot.commands.shootercommands.ToggleShooterMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.RobotCommands.*;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

 
  private final XboxController m_driveController = new XboxController(driveControllerPort);
  private final static XboxController m_techController = new XboxController(techControllerPort);

  // public final JoystickButton turnNeg = new JoystickButton(m_driveController,
  // backButtonChannel);
  // public final JoystickButton turnPos = new JoystickButton(m_driveController,
  // startButtonChannel);
  // public final JoystickButton velocityDrive = new
  // JoystickButton(m_driveController, rightTriggerChannel);
  public final JoystickButton limeTurn = new JoystickButton(m_driveController, xButtonChannel);
  // private final AxisButton limeTurn = new AxisButton(m_driveController,
  // Constants.leftTriggerChannel);
  // public final JoystickButton limeTurn2 = new JoystickButton(m_driveController,
  // yButtonChannel);
  public final JoystickButton determineDistance = new JoystickButton(m_driveController, aButtonChannel);

  public final JoystickButton m_deployCollector = new JoystickButton(m_techController, leftBumperChannel);
  public final JoystickButton m_groundCollect = new JoystickButton(m_techController, aButtonChannel);
  public final JoystickButton m_reverseGroundCollect = new JoystickButton(m_techController, yButtonChannel);
  public final JoystickButton m_conveyorRun = new JoystickButton(m_techController, xButtonChannel);
  public final JoystickButton m_reverseConveyorRun = new JoystickButton(m_techController, bButtonChannel);

  public final JoystickButton enableCompresser = new JoystickButton(m_driveController, startButtonChannel);
  public final JoystickButton disableCompresser = new JoystickButton(m_driveController, backButtonChannel);
  // public final JoystickButton m_collectToShoot = new
  // JoystickButton(m_techController, yButtonChannel);
  // public final JoystickButton m_hoodAdjust = new
  // JoystickButton(m_driveController, yButtonChannel);

  // private final POVButton shooterFar = new POVButton(m_driveController, POVU);
  // private final POVButton shooterNear = new POVButton(m_driveController, POVD);
  // public final JoystickButton shooterFar = new
  // JoystickButton(m_driveController, bButtonChannel);
  // public final JoystickButton shooterNear = new
  // JoystickButton(m_driveController, aButtonChannel);
  // public final JoystickButton shooterFarther = new
  // JoystickButton(m_driveController, yButtonChannel);
  public final JoystickButton shoot = new JoystickButton(m_driveController, rightBumperChannel);
  public final JoystickButton adjustShooter = new JoystickButton(m_driveController, leftBumperChannel);

  // public final JoystickButton m_climberPistonToggle = new
  // JoystickButton(m_techController, backButtonChannel);

  private final SequentialCommandGroup TestGrouping = new SequentialCommandGroup(
      new frc.robot.autonomous.autonroutes.TestGrouping(m_drive, m_shooter, m_collector));
  private final SequentialCommandGroup TwoBall = new SequentialCommandGroup(
      new frc.robot.autonomous.autonroutes.TwoBall(m_drive, m_shooter, m_collector));
  private final SequentialCommandGroup OneBallOffLine = new SequentialCommandGroup(
      new frc.robot.autonomous.autonroutes.OneBallOffLine(m_drive, m_shooter, m_collector));

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<String> m_driveType = new SendableChooser<>();
  public static SendableChooser<String> m_position = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    new RobotCommands();

    if(getDriveType() == "Curvy"){

       m_drive.setDefaultCommand(
      new CurvyDrive(m_drive, m_driveController));
   
    }else{

      m_drive.setDefaultCommand(
        new ZoomZoomFast(m_drive, () -> -m_driveController.getLeftY(),
            () -> m_driveController.getRightX()));

    }

    

    // m_climber.setDefaultCommand(
    // new Climb(m_climber, () -> m_techController.getRightY()));

    m_driveType.addOption("Curvy", "Curvy");

    m_chooser.addOption("TestGrouping", TestGrouping);
    m_chooser.addOption("TwoBall", TwoBall);
    m_chooser.addOption("One Ball off Line", OneBallOffLine);
    m_position.addOption("Position1", "close");
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    Shuffleboard.getTab("Autonomous").add(m_position);
  }

  private void configureButtonBindings() {

    // turnPos.whileHeld(turnTurretPos);
    // turnNeg.whileHeld(turnTurretNeg);
    // limeTurn.whileHeld(new DriveLimeTurn(m_drive), true);
    limeTurn.whileActiveContinuous(new LimeTurn(m_drive, () -> -m_driveController.getLeftY()));
    // limeTurn2.whileHeld(new Aim(m_turret, m_limelight, true));
    // velocityDrive.whileActiveContinuous(new VelocityDrive(m_drive, () ->
    // -m_driveController.getLeftY(),
    // () -> m_driveController.getRightX()));
    // m_deployCollector.whenPressed(deployCollectorCommand);
    m_deployCollector.whenPressed(deployCollectorCommand);
    m_groundCollect.whileHeld(groundCollect);
    // m_groundCollect.whenReleased(deployCollectorCommand);
    m_reverseGroundCollect.whileHeld(reverseGroundCollect);
    m_conveyorRun.whileHeld(conveyorRun);
    // m_conveyorRun.whenReleased(deployCollectorCommand);
    m_reverseConveyorRun.whileHeld(reverseConveyorRun);
    // m_collectToShoot.whileHeld(collectToShoot);
    // shooterNear.whenPressed(new AutoAdjustHood(30));
    // shooterFar.whenPressed(new AutoAdjustHood(0));
    // shooterFarther.whenPressed(new AutoAdjustHood(110));
    determineDistance.whenPressed(findDistance);

    shoot.whileHeld(shootSpee);
    adjustShooter.whenPressed(new ToggleShooterMode(m_shooter));

    enableCompresser.whenPressed(compressorEnable);
    disableCompresser.whenPressed(compressorDisable);

    // m_climberPistonToggle.whenPressed(toggleClimberSolenoids);

  }

  public String getDriveType(){
    return m_driveType.getSelected();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
