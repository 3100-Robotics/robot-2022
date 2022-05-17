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

/**
 * TwoBall - A command to collect one ball, and shoot two into the upper hub
 * @param hoodAngle The angle to set the servo to, 0-180 degrees
*/

public class TwoBall extends SequentialCommandGroup {

    public TwoBall(Drive drive, Shooter shooter, Collector collector) {

        super(

                new AutoDrive(2, 0.6, drive),
                new AutoCollectToShoot(collector, 2, 0.6, 0.6),
                new AutoTurnToAngle(175, 0.4, drive),
                new AutoRev(shooter, 2, 0.65),
                new AutoShoot(shooter, collector, 2, 0.65)

        );

    }

}
