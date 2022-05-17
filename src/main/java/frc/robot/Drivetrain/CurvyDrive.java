package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CurvyDrive extends CommandBase {
  private final Drive m_drive;
  private final XboxController m_controller;

  /**
   * Creates a new Curvature Drive.
   *
   * @see https://www.reddit.com/r/FRC/comments/80679m/what_is_curvature_drive_cheesy_drive/
   * xSpeed, speed of the robot
   * zRotation, steers the robot
   * quickTurn, allows for turn in place
   * 
   */
  public CurvyDrive(Drive subsystem, XboxController controller) {
    m_drive = subsystem;
    m_controller = controller;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {

        double xSpeed = -m_controller.getLeftY();
        double zRotation = m_controller.getRightX();
        boolean quickTurn = m_controller.getLeftBumper();

        m_drive.curvatureDrive(xSpeed, zRotation, quickTurn);
  }
}