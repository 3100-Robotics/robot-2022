package frc.robot.autonomous.autoncommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class AutoClimberArm extends CommandBase {

    public double rev;
    public double time;
    public double time2;
    public static double shooterSpeedFar = 800;


    public AutoClimberArm(double timeRun, double revs) {
    


    
        rev = revs * 2048;
        time2 = timeRun;
    }

   




    public void initialize(){
     //   new WaitCommand(time)
     time = Timer.getFPGATimestamp();
    }

   

    public void execute() {
        Climber.leftClimberMotor.set(ControlMode.Position, rev);
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