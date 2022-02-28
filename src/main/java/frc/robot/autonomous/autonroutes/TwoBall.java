package frc.robot.autonomous.autonroutes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotCommands;
import frc.robot.RobotContainer;
import frc.robot.Drivetrain.Drive;
import frc.robot.autonomous.PathGenerator;
import frc.robot.autonomous.autoncommands.AutoAdjustHood;
import frc.robot.autonomous.autoncommands.AutoCollectToShoot;
import frc.robot.autonomous.autoncommands.AutoDrive;
import frc.robot.autonomous.autoncommands.AutoRev;
import frc.robot.autonomous.autoncommands.AutoShoot;
import frc.robot.autonomous.autoncommands.AutoTurnToAngle;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class TwoBall extends SequentialCommandGroup {

    public TwoBall(Drive drive, Shooter shooter, Collector collector) {

        super(

                new AutoDrive(4, 0.6, drive),
                new AutoCollectToShoot(collector, 2, 0.6, 0.6),
                new AutoTurnToAngle(180, 0.4, drive),
                new AutoRev(shooter, 2, 0.65),
                new AutoShoot(shooter, 2, 0.65)

        );

    }

}
