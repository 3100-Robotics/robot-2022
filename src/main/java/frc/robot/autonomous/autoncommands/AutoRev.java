package frc.robot.autonomous.autoncommands;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AutoRev extends CommandBase {

    public double time;
    public double time2;
    public double top;
    public Shooter shooter;



    public AutoRev(Shooter shooter, double timeRun, double topSpeed) {
    

        this.shooter = shooter;
    
        top = topSpeed;
    
        time2 = timeRun;
    }

   




    public void initialize(){
     //   new WaitCommand(time)
     time = Timer.getFPGATimestamp();
    }

   

    public void execute() {
        shooter.shootPercentAuton(top);
    

    }
    // @Override
    public boolean isFinished() {

        if(Timer.getFPGATimestamp() >= time + time2){
            // Constants.shooterTop.set(ControlMode.PercentOutput, 0.0);
            // Constants.shooterBottom.set(ControlMode.PercentOutput, 0.0);
            return true;
            
        }
        return false;
    }


    

}