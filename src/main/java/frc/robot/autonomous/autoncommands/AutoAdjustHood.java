package frc.robot.autonomous.autoncommands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class AutoAdjustHood extends CommandBase {

    public double m_angle;
   // public static Servo hoodServo = new Servo(0);

    public AutoAdjustHood(double hoodAngle) {
        Turret.hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

        m_angle = hoodAngle;

    }

    public void initialize() {

        System.out.println("hoodAdjust");
        Turret.hoodServo.setAngle(m_angle);

    }

    public boolean isFinished() {

        return true;
    }

}