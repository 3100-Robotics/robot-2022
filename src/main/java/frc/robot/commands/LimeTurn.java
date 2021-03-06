package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.Drive;
import frc.robot.sensors.LimelightInterface;
import frc.robot.sensors.LimelightInterface.LimelightLEDMode;

public class LimeTurn extends CommandBase{

    private final Drive m_drive;
    final double STEER_K = 0.15;
    private boolean stop = false;
    private double x;
    private double v;
    private final DoubleSupplier m_forward;

    public LimeTurn(Drive subsystem, DoubleSupplier forward){

        m_forward = forward;
        m_drive = subsystem;
        addRequirements(m_drive);
    }

    public void initialize(){

        LimelightInterface.setLEDMode(LimelightLEDMode.ON);
        x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    }

    public void execute(){
  
        LimelightInterface.setLEDMode(LimelightLEDMode.ON);
        if (v < 1.0)
        {
         // return;
          stop = true;
        }
        else {
            while(Math.abs(x) > 10){

            x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
                m_drive.autoArcadeDrive(m_forward.getAsDouble(), 0.8 * Math.signum(x));

            
                

            }
            while(Math.abs(x) > 4){

                x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
                v = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
                m_drive.autoArcadeDrive(m_forward.getAsDouble(), 0.68 * Math.signum(x));

               
            }
            stop = true;
      
        }

        stop = true;
    }
    public boolean isFinished(){

        
        return stop;
    }
    public void end(){

        LimelightInterface.setLEDMode(LimelightLEDMode.OFF);

    }

}