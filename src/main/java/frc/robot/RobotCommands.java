package frc.robot;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotCommands {
    
    public final static Turret m_turret = new Turret();
    public final static Drive m_drive = new Drive();
    public final static Collector m_collector = new Collector();
    public final static Shooter m_shooter = new Shooter();
    public final static Climber m_climber = new Climber();

    

    public final static StartEndCommand shootAdjusting = new StartEndCommand(
        () -> m_shooter.shoot(),
        () -> m_shooter.stopShoot(),
    m_shooter
    );

    public final static StartEndCommand shootRPM = new StartEndCommand(
        () -> m_shooter.setVelocitySpeed(4000),
        () -> m_shooter.stopShoot(),
    m_shooter
    );

    public final static StartEndCommand turnTurretNeg = new StartEndCommand(
        () -> m_turret.turn(-0.2),
        () -> m_turret.turn(0.0),
    m_turret
    );
    
   public final static StartEndCommand turnTurretPos = new StartEndCommand(
        () -> m_turret.turn(0.2),
        () -> m_turret.turn(0.0),
    m_turret
    );

    public final static InstantCommand deployCollectorCommand = new InstantCommand(
        () -> m_collector.deployCollector(),
    m_collector
    );

    public final static InstantCommand toggleClimberSolenoids = new InstantCommand(
        () -> m_climber.toggleClimberSolenoids()
    );

    public final static InstantCommand findDistance = new InstantCommand(
    () -> m_shooter.determineObjectDist(26.5, 50),
    m_shooter
    );

    public final static InstantCommand findMountAngle = new InstantCommand(
    () -> m_shooter.determineMountingAngle(61, 26.5, 50),
    m_shooter
    );

    public final static InstantCommand adjustHood = new InstantCommand(
    () -> m_turret.adjustHood()
    );

    public final static StartEndCommand groundCollect = new StartEndCommand(
        () -> m_collector.groundCollect(0.4),
        () -> m_collector.groundCollect(0.0),
    m_collector
    );

    public final static StartEndCommand conveyorRun = new StartEndCommand(
        () -> m_collector.conveyorRun(0.4),
        () -> m_collector.conveyorRun(0.0),
    m_collector
    );

    public final static StartEndCommand collectToShoot = new StartEndCommand(
        () -> m_collector.collectToShoot(0.4),
        () -> m_collector.collectToShoot(0.0),
    m_collector
    );

}
