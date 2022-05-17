package frc.robot.autonomous.autoncommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;


/**
 * AutoAdjustHood - A command to extend/retract the servos on the hood
 * @param hoodAngle The angle to set the servo to, 0-180 degrees
*/
    
 

public class AutoAdjustHood extends CommandBase {

    public double m_angle;

    public AutoAdjustHood(double hoodAngle) {
        Turret.hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

        m_angle = hoodAngle;

    }

    public void initialize() {

        Turret.hoodServo.setAngle(m_angle);

    }

    public boolean isFinished() {

        return true;
    }

}