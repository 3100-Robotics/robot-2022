package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PneumaticsConstants;;

//TODO: CHECK TO SEE IF LEAVING THE SOLENOID IN OFF/ON CHANGES BEHAVIOR
public class Collector extends SubsystemBase {

   public static final Solenoid leftIntakeSolenoid = new
   Solenoid(PneumaticsModuleType.CTREPCM,
   PneumaticsConstants.leftIntakeSolenoidPort);
  
   public static final CANSparkMax collectorMotor = new CANSparkMax(MotorConstants.collectorMotorPort,
         MotorType.kBrushless);
   public CANSparkMax conveyorMotor = new CANSparkMax(MotorConstants.conveyorMotorPort,
         MotorType.kBrushless);

         public Collector(){

            conveyorMotor.restoreFactoryDefaults();

            conveyorMotor.setIdleMode(IdleMode.kBrake);

         }

   public void deployCollector() {

      // Set Piston to extend/retract
      leftIntakeSolenoid.toggle();
    
   }

   public void groundCollect(double speed) {

      //deployCollector();
      collectorMotor.set(speed);

   }
   public void reverseCollect(double speed) {

    //  deployCollector();
      collectorMotor.set(speed);

   }

   public void conveyorRun(double speed) {

    // System.out.println("CONVEYOR");
      conveyorMotor.set(speed);

   }

   public void collectToShoot(double speed) {

      
      collectorMotor.set(speed);
      conveyorMotor.set(speed);

   }

}
