package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    // public static final Solenoid leftClimberSolenoid = new
    // Solenoid(PneumaticsModuleType.CTREPCM,
    // PneumaticsConstants.leftClimberSolenoidPort);
    // public static final Solenoid rightClimberSolenoid = new
    // Solenoid(PneumaticsModuleType.CTREPCM,
    // PneumaticsConstants.rightClimberSolenoidPort);

    public Climber() {

        leftClimberMotor.configFactoryDefault();
        rightClimberMotor.configFactoryDefault();

        leftClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        leftClimberMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);

        leftClimberMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
        leftClimberMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);

        rightClimberMotor.follow(leftClimberMotor);

        leftClimberMotor.setInverted(false);
        rightClimberMotor.setInverted(InvertType.FollowMaster);

        SoftLimitSetup(1);

    }

    public void toggleClimberSolenoids() {

        // leftClimberSolenoid.toggle();
        // rightClimberSolenoid.toggle();

    }

    public void armActuate(double speed) {

        leftClimberMotor.set(speed);

    }

    void SoftLimitSetup(int choice) {
        if (choice == 1) {
            /*
             * not using remote 0 - turn it off to prevent remote LossOfSignal (LOS) fault.
             */
            leftClimberMotor.configRemoteFeedbackFilter(0x00, /* device ID does not matter since filter is off */
                    RemoteSensorSource.Off,
                    Constants.REMOTE_0,
                    Constants.kTimeoutMs);

            /*
             * not using remote 1 - turn it off to prevent remote LossOfSignal (LOS) fault.
             */
            leftClimberMotor.configRemoteFeedbackFilter(0x00, /* device ID does not matter since filter is off */
                    RemoteSensorSource.Off,
                    Constants.REMOTE_1,
                    Constants.kTimeoutMs);

            /* select local quadrature if using Talon FX */
            leftClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                    Constants.PID_PRIMARY,
                    Constants.kTimeoutMs);

            /* select limits */
            leftClimberMotor.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
            leftClimberMotor.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

            System.out.println("Using local integrated sensor.");

        }
    }
}
