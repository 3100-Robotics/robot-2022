package frc.robot.autonomous.autoncommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.Drive;

public class AutoDrive extends CommandBase {
    public final Drive drive;
    private final double goal, speed;
    private double time;

    public AutoDrive(double timeRun, double speed, final Drive drive) {
        this.drive = drive;
        this.goal = timeRun;
        this.speed = speed;
 

        addRequirements(drive);
    }

    public void initialize() {
        time = Timer.getFPGATimestamp();
       drive.resetEncoders();
     
    }

    public void execute() {
        // double error = drive.getLeftDistance() - drive.getRightDistance();

        // System.out.println("Error");
        // System.out.println(error);
        

        drive.autoArcadeDrive(speed, 0);

       // System.out.println(drive.getAverageEncoderDistance());

    }

    public boolean isFinished() {
        if (Timer.getFPGATimestamp() >= time + goal) {
            return true;
        }
        return false;
    }

  

}