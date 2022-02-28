package frc.robot.autonomous.autoncommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends CommandBase {

    public Shooter m_shoot;
    public Collector m_collect;
    public double time;
    public double time2;
    public double m_speed;

    public AutoShoot(Shooter subsystem, double timeRun, double speed) {

        m_shoot = subsystem;
        m_speed = speed;
        time2 = timeRun;

        addRequirements(m_shoot);
    }

    public void initialize() {
        time = Timer.getFPGATimestamp();
    }

    public void execute() {


        m_shoot.shootPercent(m_speed);
        m_collect.collectToShoot(0.8);

    }

    public boolean isFinished() {

        if (Timer.getFPGATimestamp() >= time + time2) {
            return true;
        }
        return false;
    }

}