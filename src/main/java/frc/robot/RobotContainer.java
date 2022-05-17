// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.drivetrain.CurvyDrive;
import frc.robot.drivetrain.VelocityDrive;
import frc.robot.drivetrain.ZoomZoomFast;
import frc.robot.util.AxisButton;
import frc.robot.autonomous.autoncommands.AutoAdjustHood;
import frc.robot.autonomous.autonroutes.TestGrouping;
import frc.robot.commands.Aim;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveLimeTurn;
import frc.robot.commands.LimeTurn;
import frc.robot.commands.VisionAlignDrivetrain;
import frc.robot.commands.shootercommands.ShootTable;
import frc.robot.commands.shootercommands.ToggleShooterMode;
import frc.robot.commands.shootercommands.WaitForShooterVelocity;
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
  private final XboxController m_rod = new XboxController(rodControllerPort);
  public final boolean isRod = false;

  private final SequentialCommandGroup TestGrouping = new SequentialCommandGroup(
      new frc.robot.autonomous.autonroutes.TestGrouping(m_drive, m_shooter, m_collector));
  private final SequentialCommandGroup TwoBall = new SequentialCommandGroup(
      new frc.robot.autonomous.autonroutes.TwoBall(m_drive, m_shooter, m_collector));
  private final SequentialCommandGroup OneBallOffLine = new SequentialCommandGroup(
      new frc.robot.autonomous.autonroutes.OneBallOffLine(m_drive, m_shooter, m_collector));

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Boolean> m_isRod = new SendableChooser<>();
  SendableChooser<String> m_driveType = new SendableChooser<>();
  public static SendableChooser<String> m_position = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    new RobotCommands();

    // if (getDriveType() == "Curvy") {

    // m_drive.setDefaultCommand(
    //     new CurvyDrive(m_drive, m_driveController));

    // } else {

    m_drive.setDefaultCommand(
    new ZoomZoomFast(m_drive, () -> -m_driveController.getLeftY(),
    () -> m_driveController.getRightX()));

    // }

    

    if(isRod){

      m_climber.setDefaultCommand(
        new Climb(m_climber, () -> m_rod.getRawAxis(rightXAxisChannel), false));

    }else{

      m_climber.setDefaultCommand(
    new Climb(m_climber, () -> m_techController.getRightY(), true));

    }

    

    // m_climber.setDefaultCommand(
    // new Climb(m_climber, () -> m_rod.getRawAxis(leftYAxisChannel), true));

    m_driveType.addOption("Curvy", "Curvy");
    m_isRod.addOption("Rod", isRod);

    m_chooser.addOption("TestGrouping", TestGrouping);
    m_chooser.addOption("TwoBall", TwoBall);
    m_chooser.addOption("One Ball off Line", OneBallOffLine);
    m_position.addOption("Position1", "close");
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    Shuffleboard.getTab("Autonomous").add(m_position);
  }

  private void configureButtonBindings() {

    // final JoystickButton limeTurn = new JoystickButton(m_driveController, xButtonChannel);
    // limeTurn.whileActiveContinuous(new LimeTurn(m_drive, () -> -m_driveController.getLeftY()));

    // final JoystickButton limeTurn2 = new JoystickButton(m_driveController, yButtonChannel);
    // limeTurn2.whileActiveContinuous(new VisionAlignDrivetrain(m_limelight, m_drive));

    // final JoystickButton determineDistance = new JoystickButton(m_driveController, aButtonChannel);
    // determineDistance.whenPressed(printDistance);


    final JoystickButton enableCompresser = new JoystickButton(m_driveController, startButtonChannel);
    enableCompresser.whenPressed(compressorEnable);
    final JoystickButton disableCompresser = new JoystickButton(m_driveController, backButtonChannel);
    disableCompresser.whenPressed(compressorDisable);

    final JoystickButton shoot = new JoystickButton(m_driveController, rightBumperChannel);
    //shoot.whileHeld(new ShootTable(m_shooter, m_turret));
    shoot.whileHeld(shootSpee);
    // final JoystickButton adjustShooter = new JoystickButton(m_driveController, leftBumperChannel);
    // adjustShooter.whenPressed(new ToggleShooterMode(m_shooter));

    if(isRod){

      final JoystickButton m_deployCollector = new JoystickButton(m_rod, leftBumperChannel);
      m_deployCollector.whenPressed(deployCollectorCommand);
      final JoystickButton m_groundCollect = new JoystickButton(m_rod, aButtonChannel);
      m_groundCollect.whileHeld(groundCollect);
      final JoystickButton m_reverseGroundCollect = new JoystickButton(m_rod, yButtonChannel);
      m_reverseGroundCollect.whileHeld(reverseGroundCollect);
      final JoystickButton m_conveyorRun = new JoystickButton(m_rod, xButtonChannel);
      m_conveyorRun.whileHeld(conveyorRun);
      final JoystickButton m_reverseConveyorRun = new JoystickButton(m_rod, bButtonChannel);
      m_reverseConveyorRun.whileHeld(reverseConveyorRun);
      final JoystickButton m_reverseShooter = new JoystickButton(m_rod, rightBumperChannel);
      m_reverseShooter.whileHeld(reverseShooter);
      final AxisButton m_climberPistonToggle = new AxisButton(m_rod, leftTriggerChannel);
      m_climberPistonToggle.whileActiveOnce(toggleClimberSolenoids);

      final POVButton m_reset = new POVButton(m_rod, POVU);
       m_reset.whenPressed(resetClimberSensors);

    }else{

      final JoystickButton m_deployCollector = new JoystickButton(m_techController, leftBumperChannel);
      m_deployCollector.whenPressed(deployCollectorCommand);
      final JoystickButton m_groundCollect = new JoystickButton(m_techController, aButtonChannel);
      m_groundCollect.whileHeld(groundCollect);
      final JoystickButton m_reverseGroundCollect = new JoystickButton(m_techController, yButtonChannel);
      m_reverseGroundCollect.whileHeld(reverseGroundCollect);
      final JoystickButton m_conveyorRun = new JoystickButton(m_techController, xButtonChannel);
      m_conveyorRun.whileHeld(conveyorRun);
      final JoystickButton m_reverseConveyorRun = new JoystickButton(m_techController, bButtonChannel);
      m_reverseConveyorRun.whileHeld(reverseConveyorRun);
      final JoystickButton m_reverseShooter = new JoystickButton(m_techController, rightBumperChannel);
      m_reverseShooter.whileHeld(reverseShooter);
  
     
  
      final AxisButton m_climberPistonToggle = new AxisButton(m_techController, leftTriggerChannel);
      m_climberPistonToggle.whileActiveOnce(toggleClimberSolenoids);

      final POVButton m_reset = new POVButton(m_techController, POVU);
       m_reset.whenPressed(resetClimberSensors);
    }

  

   // shoot.whileHeld(new WaitForShooterVelocity(m_shooter, ShooterConstants.DIST_TO_RPM_TABLE.get(1.0)));
   //shoot.whileHeld(shootSpee);
   //shoot.whileHeld(new ShootTable(m_shooter, m_turret));
    

    

  }

  public String getDriveType() {
    return m_driveType.getSelected();
  }
  public Boolean getTechControllerType(){
    return m_isRod.getSelected();
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
