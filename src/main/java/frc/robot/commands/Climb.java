package frc.robot.commands;

import java.util.function.DoubleSupplier;

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
    private final boolean isAxis;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param rotation  The control input for turning
     */
    public Climb(Climber subsystem, DoubleSupplier motor, Boolean isAxis) {// , BooleanSupplier piston) {
        m_climber = subsystem;
        // m_piston = piston;
        m_motor = motor;
        this.isAxis = isAxis;

        addRequirements(m_climber);
    }

    double DeadbandRod(double value) {
        /* Upper deadband */
        if (value >= +0.01)
          return value;
    
        /* Lower deadband */
        if (value <= -0.01)
          return -1 * (value + 1);
    
        /* Outside deadband */
        return 0;
      }
      double DeadbandAxis(double value) {
        /* Upper deadband */
        if (value >= +0.1)
          return value;
    
        /* Lower deadband */
        if (value <= -0.1)
          return value;
    
        /* Outside deadband */
        return 0;
      }

    public void execute() {

       // System.out.println("Climb: " + Deadband(m_motor.getAsDouble()));
       if(isAxis){
        m_climber.armActuate(DeadbandAxis(m_motor.getAsDouble()));
       }else{
        m_climber.armActuate(DeadbandRod(m_motor.getAsDouble()));
       }

    }
}