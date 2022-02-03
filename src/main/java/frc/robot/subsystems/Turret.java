package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

//import static frc.robot.Constants.*;

public class Turret extends SubsystemBase{

    public PIDController turnController;
    private double x;

    public Turret(){

        turnController = new PIDController(0.01, 0, 0);
        x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    }
    
    public void turn(final double speed){

        MotorConstants.turretMotor.set(speed);

    }
    public void limeTurn(){

        x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    }

   

}
