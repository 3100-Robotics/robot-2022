package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PneumaticsConstants;;

//TODO: CHECK TO SEE IF LEAVING THE SOLENOID IN OFF/ON CHANGES BEHAVIOR
public class Collector extends SubsystemBase {

   // public static final Solenoid leftIntakeSolenoid = new
   // Solenoid(PneumaticsModuleType.CTREPCM,
   // PneumaticsConstants.leftIntakeSolenoidPort);
   // public static final Solenoid rightIntakeSolenoid = new
   // Solenoid(PneumaticsModuleType.CTREPCM,
   // PneumaticsConstants.rightIntakeSolenoidPort);
   public static final CANSparkMax collectorMotor = new CANSparkMax(MotorConstants.collectorMotorPort,
         MotorType.kBrushless);
   public static final CANSparkMax conveyorMotor = new CANSparkMax(MotorConstants.conveyorMotorPort,
         MotorType.kBrushless);

   public static void deployCollector() {

      // Set Piston to extend/retract
      // leftIntakeSolenoid.toggle();
      // rightIntakeSolenoid.toggle();
   }

   public void groundCollect(double speed) {

      collectorMotor.set(speed);

   }

   public void conveyorRun(double speed) {

      conveyorMotor.set(speed);

   }

   public void collectToShoot(double speed) {

      collectorMotor.set(speed);
      conveyorMotor.set(speed);

   }

}
