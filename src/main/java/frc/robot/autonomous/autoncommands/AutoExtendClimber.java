package frc.robot.autonomous.autoncommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class AutoExtendClimber extends CommandBase {

   

    public AutoExtendClimber() {
      

    }

    public void initialize() {

        Climber.toggleClimberSolenoids();

    }

    public boolean isFinished() {

        return true;
    }

}