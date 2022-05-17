package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PneumaticsConstants;

//two pistons when I say piston
//two falcons when I say falcon
//slave and master

//Piston should be extended all the time
//Extends the arm using a falcon to a set-point
//Drive backwards
//Pulls in the arm all the way, rotational hook sets
//Arms partially extend STEP 4
//Piston retracts to get arm aligned
//Extends the arm the same as step 1 or possibly more MAX will figure it out
//Piston extends
//Arm retracts, rotational hook sets
//Repeat from STEP 4

public class Climber extends SubsystemBase {

    public static WPI_TalonFX leftClimberMotor = new WPI_TalonFX(MotorConstants.leftClimberMotorPort);
    public static WPI_TalonFX rightClimberMotor = new WPI_TalonFX(MotorConstants.rightClimberMotorPort);

    public static final Solenoid leftClimberSolenoid = new
    Solenoid(PneumaticsModuleType.CTREPCM,
    PneumaticsConstants.climberSolenoidPort);
    

    public Climber() {

        leftClimberMotor.configFactoryDefault();
        rightClimberMotor.configFactoryDefault();

       // leftClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
       // rightClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);

       //  leftClimberMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
         rightClimberMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
       //  leftClimberMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);
         rightClimberMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);


        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);

        leftClimberMotor.follow(rightClimberMotor);

        //enabled, Limit(amp), Trigger Threshold(amp), Trigger Threshold Time(s)
        //rightClimberMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 0, 250, 1.0));

        // leftClimberMotor.setInverted(false);
        // rightClimberMotor.setInverted(InvertType.FollowMaster);

        SoftLimitSetup();

    //     rightClimberMotor.setSensorPhase(true);
    //    rightClimberMotor.setInverted(true);

        leftClimberMotor.setSelectedSensorPosition(0);
        rightClimberMotor.setSelectedSensorPosition(0);

    }

    @Override
    public void periodic() {
       // System.out.println("left " + leftClimberMotor.getSelectedSensorPosition() / 2048);
       
       //  System.out.println(rightClimberMotor.getMotorOutputVoltage());
    }

    public void toggleClimberSolenoids() {

      System.out.println("Pistons");
         leftClimberSolenoid.toggle();
        

    }

    public void armActuate(double speed) {

      //System.out.println("right " + rightClimberMotor.getSelectedSensorPosition() / 2048);
      //  leftClimberMotor.set(speed);
        rightClimberMotor.set(-speed);

    }

    public void reset(){

      System.out.println("RESET");

        leftClimberMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
        rightClimberMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);

    }

    public boolean checkCurrent(){


        if(rightClimberMotor.getMotorOutputVoltage() >= 11){

          return true;
        }else{

          return false;
        }

    }

    void SoftLimitSetup() {
      
          

            /* select local quadrature if using Talon FX */
            leftClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                    Constants.PID_PRIMARY,
                    Constants.kTimeoutMs);
            rightClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                    Constants.PID_PRIMARY,
                    Constants.kTimeoutMs);
           

            /* select limits */
             leftClimberMotor.configForwardSoftLimitThreshold(ClimberConstants.kLeftForwardSoftLimit_Quad, Constants.kTimeoutMs);
             rightClimberMotor.configForwardSoftLimitThreshold(ClimberConstants.kRightForwardSoftLimit_Quad, Constants.kTimeoutMs);
             leftClimberMotor.configReverseSoftLimitThreshold(ClimberConstants.kLeftReverseSoftLimit_Quad, Constants.kTimeoutMs);
             rightClimberMotor.configReverseSoftLimitThreshold(ClimberConstants.kRightReverseSoftLimit_Quad, Constants.kTimeoutMs);

        
    }
}
