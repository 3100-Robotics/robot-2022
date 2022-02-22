package frc.robot.autonomous.autonroutes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Drivetrain.Drive;
import frc.robot.autonomous.PathGenerator;
import frc.robot.autonomous.autoncommands.AutoAdjustHood;
import frc.robot.autonomous.autoncommands.AutoShoot;
import frc.robot.subsystems.Shooter;

public class TestGrouping extends SequentialCommandGroup {

    public TestGrouping(Shooter shooter) {

       
         

                super(

                    PathGenerator.generateTrajectory("Straight")
                //    //0-180
                //    new AutoAdjustHood(60.0),
                //    new AutoShoot(shooter, 2, 5000.0)

                    );

         
                
        // new RobotCommands().retractCollector2,
        // new AutoRev(4, 0.25, 0.6),
        // new AutoShoot(3, 0.25, 0.6),
        // new AutonDrive(1 * Constants.moveConstant, 0.8, drive)//,
        // new TurnToAngle2(75, 0.8, drive)

   

    }

}
