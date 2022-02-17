package frc.robot.autonomous.autonroutes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.autoncommands.AutoClimberArm;
import frc.robot.autonomous.autoncommands.AutoExtendClimber;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;

public class AutoClimber2 extends SequentialCommandGroup { 

    public AutoClimber2(){

        super(
            
//double timeRun, double revs
        new AutoClimberArm(6, 0),
        new AutoClimberArm(3, 2),
        new AutoExtendClimber(),
        new AutoClimberArm(6, 6),
        new AutoExtendClimber(),
        new AutoClimberArm(6, 0),
        new AutoClimberArm(3, 2),
        new AutoExtendClimber(),
        new AutoClimberArm(6, 6),
        new AutoExtendClimber(),
        new AutoClimberArm(6, 0)

        );


    }
    
}
