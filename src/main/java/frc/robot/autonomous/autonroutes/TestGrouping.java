package frc.robot.autonomous.autonroutes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.PathGenerator;

public class TestGrouping extends SequentialCommandGroup {

    public TestGrouping() {

        super(
                PathGenerator.generateTrajectory("Testing A Curve")
        // new RobotCommands().retractCollector2,
        // new AutoRev(4, 0.25, 0.6),
        // new AutoShoot(3, 0.25, 0.6),
        // new AutonDrive(1 * Constants.moveConstant, 0.8, drive)//,
        // new TurnToAngle2(75, 0.8, drive)

        );

    }

}
