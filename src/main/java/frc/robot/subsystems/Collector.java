package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: CHECK TO SEE IF LEAVING THE SOLENOID IN OFF/ON CHANGES BEHAVIOR
public class Collector extends SubsystemBase {

   public static final Solenoid leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
   public static final Solenoid rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
   public static final CANSparkMax collectorMotor = new CANSparkMax(3, MotorType.kBrushless);


   public void deployCollector() {

    // Set Piston to extend/retract
    leftIntakeSolenoid.toggle();
    rightIntakeSolenoid.toggle();
}


}
