package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Code should inline a command this simple with
 * {@link edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class Climb extends CommandBase {
    private final Climber m_climber;
    // private final BooleanSupplier m_piston;
    private final DoubleSupplier m_motor;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param rotation  The control input for turning
     */
    public Climb(Climber subsystem, DoubleSupplier motor) {// , BooleanSupplier piston) {
        m_climber = subsystem;
        // m_piston = piston;
        m_motor = motor;

        addRequirements(m_climber);
    }

    public void execute() {

        m_climber.armActuate(m_motor.getAsDouble());

    }
}