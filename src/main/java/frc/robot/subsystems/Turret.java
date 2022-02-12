package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

//import static frc.robot.Constants.*;

public class Turret extends SubsystemBase {

    public PIDController turnController;
    public final static Servo hoodServo = new Servo(MotorConstants.hoodServoPort);
    private double x;
    private ShuffleboardTab tab = Shuffleboard.getTab("Turret");
    private NetworkTableEntry hoodAngle = tab.addPersistent("Hood Angle", 0)
            .withProperties(Map.of("min", 0, "max", 180))
            .getEntry();
    private double hoodAngleNumber;

    public void periodic() {

        hoodAngle.setNumber(hoodAngleNumber);
        // x =
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    }

    public Turret() {

        turnController = new PIDController(0.01, 0, 0);
        x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    }

    public void turn(final double speed) {

        MotorConstants.turretMotor.set(speed);

    }

    public void limeTurn() {

        x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    }

    //0-180
    public void adjustHood() {

        hoodServo.setAngle(hoodAngleNumber);

    }
    public void adjustHoodAuton(double angle) {

        hoodServo.setAngle(angle);

    }

}
