package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.Drive;
import frc.robot.subsystems.Turret;

public class DriveLimeTurn extends CommandBase {

  private final Drive m_drive;
  final double STEER_K = 0.15;
  private boolean stop = false;
  private double x;
  private double v;
  private double adjustment = 0.0;
  private double prev_tx = 1.0;
  private double steering_factor = 0.25;
  public boolean stopSteer = false;
  private boolean newPIDLoop = false;
  private double Kp = -0.1;
  private double min_command = 0.05;
  public PIDController turnController;

  private double reflectiveTarget, horizantal_angle, vertical_angle;
  double forward_output = 0;
  double turn_output = 0;

  private final double max_auto_output = 0.5;
  private final double kP_dist = 0.0; // First tune the turn PD loop
  private final double dist_tol = 0.5;
  private final double desired_vert_angle = 10;

  private final double kP_turn = 0.025;
  private final double kD_turn = kP_turn * 10;
  private final double angle_tol = 0.5;
  private final double max_anglular_vel = 15; // Deg/Sec
  private final double max_anglular_accel = 10; // Deg/Sec^2

  private final ProfiledPIDController m_controller = new ProfiledPIDController(kP_turn, 0.0, kD_turn,
      new TrapezoidProfile.Constraints(max_anglular_vel, max_anglular_accel));

  public DriveLimeTurn(Drive subsystem) {

    m_drive = subsystem;
    addRequirements(m_drive);
  }

  public void initialize() {

    turnController = new PIDController(0.01, 0.01, 0);

    // pidController.setPID(0.015, 0, 0);
    // pidController.setTolerance(0.01);
  }

  public void execute() {

    horizantal_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    vertical_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    reflectiveTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    // double heading_error = -x;
    // double steering_adjust = 0.0;

    // if (v == 1 && !stopSteer){

    // adjustment = pidController.calculate(x);
    // prev_tx = x;

    // if (!newPIDLoop) {
    // newPIDLoop = true;
    // pidController.setSetpoint(Math.signum(prev_tx) );
    // }

    // }else{

    // newPIDLoop = false;
    // pidController.reset();
    // adjustment = Math.signum(prev_tx) * steering_factor;

    // }

    // if (Math.abs(x) < 1.0 && Math.abs(prev_tx) < 1.0) stopSteer = true;
    // else stopSteer = false;
    // if(stopSteer) {
    // adjustment = 0;
    // }

    // adjustment = Math.signum(x) * Math.min(Math.abs(adjustment), 0.5);

    // m_turret.turn(adjustment);
    // SmartDashboard.putNumber("Adjustment", adjustment);

    // left_command += steering_adjust;
    // right_command -= steering_adjust;
    if (reflectiveTarget == 1.0) {

      // Have a deadband where we are close enough
      if (Math.abs(horizantal_angle) > angle_tol) {
        // Get PID controller output
        turn_output += m_controller.calculate(horizantal_angle);
      }

      // forward_output - turn_output
      m_drive.arcadeDrive(0, deadband(turn_output));

    }
    
    // m_turret.turn(-turnController.calculate(x, 0));
    else {

      stop = true;
      //if (reflectiveTarget == 1.0) {
      // System.out.println("Not in Sight");

      // while(Math.abs(x) > 10){

      // x =
      // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      // v =
      // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      // m_turret.turn(0.3 * Math.signum(x));

      // System.out.println("faster");

      // }
      // while(Math.abs(x) > 2 && Math.abs(x) < 10){

      // x =
      // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      // v =
      // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      // m_turret.turn(0.1 * Math.signum(x));
      // System.out.println("slower");

      // }
    //}
    //else{
      // stop = true;
      //}
    }
  }

  // // stop = true;
  // }

  public boolean isFinished() {
    // System.out.println("finished");
    return stop;
  }

  public double deadband(double a) {
    return Math.abs(a) < max_auto_output ? a : Math.copySign(max_auto_output, a);
  }

}