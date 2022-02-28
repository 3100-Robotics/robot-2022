package frc.robot.autonomous.autoncommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class AutoExtendClimber extends CommandBase {
    
   
    public Climber m_climber;

    public AutoExtendClimber(Climber subsystem) {
      
        m_climber = subsystem;

    }

    public void initialize() {

        m_climber.toggleClimberSolenoids();

    }

    public boolean isFinished() {

        return true;
    }

}