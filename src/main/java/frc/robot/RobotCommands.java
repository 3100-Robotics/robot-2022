package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.drivetrain.Drive;
import frc.robot.autonomous.PathGenerator;
import frc.robot.sensors.LimelightInterface;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotCommands {
    
    public final static Turret m_turret = new Turret();
    public final static Drive m_drive = new Drive();
    public final static Collector m_collector = new Collector();
    public final static Shooter m_shooter = new Shooter();
    public final static Climber m_climber = new Climber();
    public final static LimelightInterface m_limelight = new LimelightInterface();
    public final static PathGenerator m_pathGenerator = new PathGenerator(m_drive);

    static Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    // public final static StartEndCommand shootAdjusting = new StartEndCommand(
    //     () -> m_shooter.shoot(),
    //     () -> m_shooter.stopShoot(),
    // m_shooter
    // );

    public final static InstantCommand shootRPM = new InstantCommand(
        () -> m_shooter.setRPM(4000)
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
    () -> LimelightInterface.determineObjectDist(49, 75),
    m_shooter
    );


    public final static InstantCommand adjustHood = new InstantCommand(
        //60-108
    () -> m_turret.adjustHoodAuton(120)
    );
    public final static InstantCommand adjustHood2 = new InstantCommand(
        //60-108
    () -> m_turret.adjustHoodAuton(90)
    );

    public final static StartEndCommand groundCollect = new StartEndCommand(
        () -> m_collector.groundCollect(0.9),
        () -> m_collector.groundCollect(0.0)
    );
    public final static StartEndCommand reverseGroundCollect = new StartEndCommand(
        () -> m_collector.reverseCollect(-0.9),
        () -> m_collector.reverseCollect(0.0)
    );

    public final static StartEndCommand conveyorRun = new StartEndCommand(
        () -> m_collector.conveyorRun(1.0),
        () -> m_collector.conveyorRun(0.0)
    );
    public final static StartEndCommand reverseConveyorRun = new StartEndCommand(
        () -> m_collector.conveyorRun(-0.8),
        () -> m_collector.conveyorRun(0.0)
    );

    public final static StartEndCommand collectToShoot = new StartEndCommand(
        () -> m_collector.collectToShoot(0.6),
        () -> m_collector.collectToShoot(0.0),
    m_collector
    );

    public final static StartEndCommand shootSpee = new StartEndCommand(
        () -> m_shooter.shootPercent(),
        () -> m_shooter.stopShoot(),
    m_shooter
    );
    public final static StartEndCommand reverseShooter = new StartEndCommand(
        () -> m_shooter.shootPercentAuton(-0.5),
        () -> m_shooter.stopShoot(),
    m_shooter
    );
    // public final static StartEndCommand testShoot = new StartEndCommand(
    //     () -> m_shooter.shootPercentAuton(0.9),
    //     () -> m_shooter.shootPercentAuton(0.0),
    // m_shooter
    // );
    public final static InstantCommand testShoot = new InstantCommand(
        () -> m_shooter.shootPercentAuton(0.9)
    );
    

    public final static StartEndCommand setOpenLoop = new StartEndCommand(
        () -> m_shooter.setOpenLoop(0.5),
        () -> m_shooter.stopShoot(),
    m_shooter
    );

    

    // public final static StartEndCommand shootVel = new StartEndCommand(
    //     () -> m_shooter.setVelocitySpeed(5600),
    //     () -> m_shooter.stopShoot(),
    // m_shooter
    // );

    public final static InstantCommand stopShooting = new InstantCommand(
        () -> m_shooter.stopShoot()
      
    );

    public final static InstantCommand compressorDisable = new InstantCommand(
    () -> pcmCompressor.disable()
    );
    public final static InstantCommand compressorEnable = new InstantCommand(
    () -> pcmCompressor.enableDigital()
    );
    public final static InstantCommand resetClimberSensors = new InstantCommand(
    () -> m_climber.reset(),
    m_climber
    );
    public final static InstantCommand printDistance = new InstantCommand(
    () -> m_shooter.print()
    );
    

  

}
