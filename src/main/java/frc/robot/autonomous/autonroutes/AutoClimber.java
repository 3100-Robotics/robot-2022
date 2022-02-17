package frc.robot.autonomous.autonroutes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.autoncommands.AutoClimberArm;

public class AutoClimber extends SequentialCommandGroup { 

    public AutoClimber(){

        super(
            

        new AutoClimberArm(6, 6)



        );


    }
    
}
