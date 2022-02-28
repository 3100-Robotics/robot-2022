package frc.robot.autonomous.autoncommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain.Drive;

public class AutoDrive extends CommandBase {
    public final Drive drive;
    private final double goal, speed;

    public AutoDrive(final double goalDistance, double speed, final Drive drive) {
        this.drive = drive;
        this.goal = goalDistance;
        this.speed = speed;
 

        addRequirements(drive);
    }

    public void initialize() {
       drive.resetEncoders();
     
    }

    public void execute() {
        // double error = drive.getLeftDistance() - drive.getRightDistance();

        // System.out.println("Error");
        // System.out.println(error);
        

        drive.autoArcadeDrive(speed, 0);

    }

    public boolean isFinished() {
        System.out.println(drive.getAverageEncoderDistance());
        return (Math.abs(drive.getAverageEncoderDistance()) >= goal);
    }

  

}