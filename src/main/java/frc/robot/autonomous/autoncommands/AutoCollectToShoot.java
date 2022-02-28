package frc.robot.autonomous.autoncommands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class AutoCollectToShoot extends CommandBase {

    public Collector m_collecter;
    public double time;
    public double time2;
    public double m_convey;
    public double m_collect;

    public AutoCollectToShoot(Collector subsystem, double timeRun, double conveySpeed, double collectSpeed) {

        m_collecter = subsystem;
        m_convey = conveySpeed;
        m_collect = collectSpeed;
        time2 = timeRun;

        addRequirements(m_collecter);
    }

    public void initialize() {
        time = Timer.getFPGATimestamp();
    }

    public void execute() {

        m_collecter.conveyorRun(m_convey);
        m_collecter.groundCollect(m_collect);

      //  Shooter.setVelocitySpeed(m_speed);

    }

    public boolean isFinished() {

        if (Timer.getFPGATimestamp() >= time + time2) {
            return true;
        }
        return false;
    }

}