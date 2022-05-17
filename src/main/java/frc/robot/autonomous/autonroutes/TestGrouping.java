package frc.robot.autonomous.autonroutes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotCommands;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.Drive;
import frc.robot.autonomous.PathGenerator;
import frc.robot.autonomous.autoncommands.AutoAdjustHood;
import frc.robot.autonomous.autoncommands.AutoCollectToShoot;
import frc.robot.autonomous.autoncommands.AutoDrive;
import frc.robot.autonomous.autoncommands.AutoRev;
import frc.robot.autonomous.autoncommands.AutoShoot;
import frc.robot.autonomous.autoncommands.AutoTurnToAngle;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class TestGrouping extends SequentialCommandGroup {

    public TestGrouping(Drive drive, Shooter shooter, Collector collector) {

       
         

                super(

                new AutoDrive(4, 0.6, drive),
                new AutoCollectToShoot(collector, 2, 0.6, 0.6),
                new AutoTurnToAngle(180, 0.4, drive),
                new AutoRev(shooter, 2, 0.65)
             //   new AutoShoot(shooter, 2, 0.65)

                  //  PathGenerator.generateTrajectory("Testing A Curve")
                   //60-108
                //    new AutoAdjustHood(90),
                //    new ShootCommand(shooter, collector, 0.65),
                //    RobotCommands.stopShooting
                   
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
