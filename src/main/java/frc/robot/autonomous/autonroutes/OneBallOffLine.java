package frc.robot.autonomous.autonroutes;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drivetrain.Drive;
import frc.robot.autonomous.autoncommands.AutoDrive;
import frc.robot.autonomous.autoncommands.AutoRev;
import frc.robot.autonomous.autoncommands.AutoShoot;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

/**
 * OneBallOffLine - A command group to shoot a ball into the upper hub and move off the tarmac line
*/

public class OneBallOffLine extends SequentialCommandGroup {

    public OneBallOffLine(Drive drive, Shooter shooter, Collector collector) {

        super(
            new AutoRev(shooter, 3, 0.75),
            new AutoShoot(shooter, collector, 2, 0.75),
            new AutoDrive(2, -0.6, drive)
                

        );

    }

}
