package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.drivetrain.Drive;
import frc.robot.sensors.LimelightInterface;

public class VisionAlignDrivetrain extends CommandBase {
    private final LimelightInterface vision;
    private final Drive drive;
    // private final Joystick leftJoystick;
    // private final Joystick rightJoystick;
    double leftPower;
    double rightPower;
    double backOfRange;

    private boolean seesTarget;

    private double p;
    private double frictionConstant;

    public VisionAlignDrivetrain(LimelightInterface vision_subsystem, Drive drive_subsystem){
        vision = vision_subsystem;
        drive = drive_subsystem;
        // this.leftJoystick = leftJoystick;
        // this.rightJoystick = rightJoystick;
        seesTarget = false;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        frictionConstant = LimelightConstants.FRICTION_LOW;
        p = LimelightConstants.VISION_TURNING_P_LOW;
	//vision.limelightOn();
    }

    @Override
    public void execute() {
	//vision.limelightOn();
        double tx = vision.getTargetHorizAngle();
        double steering_adjust = 0.0f;

        seesTarget = vision.getTargetArea() != 0.0;

        if(seesTarget) {
            if (tx > 1.0) {
                steering_adjust = p * tx;
            } else if (tx < 1.0) {    //robot needs to turn left
                steering_adjust = p * tx;
            }
            leftPower = steering_adjust;
            rightPower = -steering_adjust;
            drive.tankDrive(-leftPower, rightPower);
        }
        else {
            drive.tankDrive(0, 0);
        }
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) {
	   // vision.limelightOff();
    }
}