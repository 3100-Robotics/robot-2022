package frc.robot.autonomous.autoncommands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAdjustHood extends CommandBase {

    public double m_angle;
    public static Servo hoodServo = new Servo(6);

    public AutoAdjustHood(double hoodAngle) {
        hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

        m_angle = hoodAngle;

    }

    public void initialize() {

        hoodServo.setAngle(m_angle);

    }

    public boolean isFinished() {

        return true;
    }

}