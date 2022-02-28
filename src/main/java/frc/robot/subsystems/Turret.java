package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;

//import static frc.robot.Constants.*;

public class Turret extends SubsystemBase {

    public PIDController turnController;
    public final static Servo hoodServo = new Servo(MotorConstants.hoodServoPort);
    private double x;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, rotations;
    private ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    private NetworkTableEntry hoodAngle = tab.addPersistent("Hood Angle", 0)
            .withProperties(Map.of("min", 0, "max", 180))
            .getEntry();
    private double hoodAngleNumber;
    public final static CANSparkMax turretMotor = new CANSparkMax(MotorConstants.turretMotorPort, MotorType.kBrushless);
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;

    public void periodic() {

    hoodAngleNumber = hoodAngle.getDouble(0);

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
        // x =
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    }
    }

    public Turret() {

        turretMotor.restoreFactoryDefaults();

        turnController = new PIDController(0.01, 0, 0);

        hoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        m_encoder = turretMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

        x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, TurretConstants.turretForwardSoftLimit);
        turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, TurretConstants.turretReverseSoftLimit);

        
        m_pidController = turretMotor.getPIDController();
       // m_pidController.setFeedbackDevice(m_encoder);

        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    m_encoder.setPosition(0);

    }

    public void turn(final double speed) {

        turretMotor.set(speed);

    }

    public void turn180() {

        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);


        
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());

    }

    public void setPosition(double position){

        m_encoder.setPosition(position);

    }

    public void getPosition(){

        m_encoder.getPosition();

    }
    public void checkEnd(){

        if (m_encoder.getPosition() >= (TurretConstants.turretForwardSoftLimit - TurretConstants.forwardTolerance) || m_encoder.getPosition() == TurretConstants.turretReverseSoftLimit + TurretConstants.reverseTolerance){

            System.out.println("At Endpoint");
            m_encoder.setPosition(0);

        }else{

            System.out.println("Not at an end point");

        }

    }

    public void stop(){

        turretMotor.stopMotor();

    }

    //0-180
    public void adjustHood() {

        hoodServo.setAngle(hoodAngleNumber);

    }
    public static void adjustHoodAuton(double angle) {

        System.out.println("Adjusting");
        hoodServo.setAngle(angle);

    }
    

}
