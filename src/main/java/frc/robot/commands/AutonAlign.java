package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain.Drive;

public class AutonAlign extends CommandBase{

    private final Drive m_drive;
    final double STEER_K = 0.15;
    private boolean stop = false;
    private double x;
    private double v;

    public AutonAlign(Drive subsystem){

        m_drive = subsystem;
        addRequirements(m_drive);
    }

    public void initialize(){

        x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    }

    public void execute(){
  
        if (v < 1.0)
        {
          return;
        }
        else {
            while(Math.abs(x) > 10){

            x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
                m_drive.autoArcadeDrive(0.0, 0.8 * Math.signum(x));

            


            }
            while(Math.abs(x) > 2){

                x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
                v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
                m_drive.autoArcadeDrive(0.0, 0.68 * Math.signum(x));

               
            }
            stop = true;
      
        }

        stop = true;
    }
    public boolean isFinished(){

        return stop;
    }

}