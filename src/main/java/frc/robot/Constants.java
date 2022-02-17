// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

   // O I - C O N S T A N T S //
   public static int driveControllerPort = 0;
   public static int techControllerPort = 1;
   public static double kDriveWheelTrackWidthMeters;

   public final static Gains kGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
   public final static Gains kGains_Turning = new Gains(0.10, 0.0, 0.0, 0.0, 200, 1.00);
   public final static Gains kGains_Velocit = new Gains(0.1, 0.001, 5, 1023.0 / 20660.0, 300, 1.00);
   public final static Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 20660.0, 400, 1.00);

   public final static double kNeutralDeadband = 0.001;
   public final static int kTimeoutMs = 30;
   public final static int kSensorUnitsPerRotation = 2048;
   public final static int kCANTimeoutMs = 10;
   

   // FIND OUT
   public static final double shooterGearRatio = 0.0;
   public final static double shooterEncoderRatio = kSensorUnitsPerRotation / 600 / shooterGearRatio;
   public static final double MAX_RPM = 6380 * shooterGearRatio;

   public final static int kForwardSoftLimit_Quad = +2048 * 5; /* 5 rotations assuming FX Integrated Sensor */
   public final static int kReverseSoftLimit_Quad = -2048 * 5; /* 5 rotations assuming FX Integrated Sensor */

   public final static int REMOTE_0 = 0;
   public final static int REMOTE_1 = 1;
   /*
    * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
    * is auxiliary
    */
   public final static int PID_PRIMARY = 0;
   public final static int PID_TURN = 1;
   /*
    * Firmware currently supports slots [0, 3] and can be used for either PID Set
    */
   public final static int SLOT_0 = 0;
   public final static int SLOT_1 = 1;
   public final static int SLOT_2 = 2;
   public final static int SLOT_3 = 3;
   /* ---- Named slots, used to clarify code ---- */
   public final static int kSlot_Distanc = SLOT_0;
   public final static int kSlot_Turning = SLOT_1;
   public final static int kSlot_Velocit = SLOT_2;
   public final static int kSlot_MotProf = SLOT_3;

   public static final class TrajectoryConstants {

      public static final double kTrackwidthMeters = 0.66;
      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
         kTrackwidthMeters);
      public static final double ksVolts = 0.68;
      public static final double kvVoltSecondsPerMeter = 2.23;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
   // 2.3
   public static final double kPDriveVel = 2.35;

   public static final double kMaxSpeedMetersPerSecond = 3;
   public static final double kMaxAccelerationMetersPerSecondSquared = 3;

   public static final double kRamseteB = 2;
   public static final double kRamseteZeta = 0.7;

   }

   public static final class OIConstants {
      // === XBOX CHANNELS === //
      // AXES
      public static final int leftXAxisChannel = 0;
      public static final int leftYAxisChannel = 1;
      public static final int leftTriggerChannel = 2;
      public static final int rightTriggerChannel = 3;
      public static final int rightXAxisChannel = 4;
      public static final int rightYAxisChannel = 5;

      // BUTTONS
      public static final int aButtonChannel = 1;
      public static final int bButtonChannel = 2;
      public static final int xButtonChannel = 3;
      public static final int yButtonChannel = 4;

      public static final int leftBumperChannel = 5;
      public static final int rightBumperChannel = 6;

      public static final int backButtonChannel = 7;
      public static final int startButtonChannel = 8;

      public static final int POVU = 0;
      public static final int POVR = 90;
      public static final int POVD = 180;
      public static final int POVL = 270;
   }

   public static final class MotorConstants {

      

      
      public final static int frontLeftDrivePort = 1;
      public final static int backLeftDrivePort = 2;
      public final static int frontRightDrivePort = 3;
      public final static int backRightDrivePort = 4;
      public final static int collectorMotorPort = 5;
      public final static int conveyorMotorPort = 6;
      public final static int hoodServoPort = 7;
      public final static int turretMotorPort = 8;
      public final static int shooterMotorPort = 9;
      public final static int leftClimberMotorPort = 10;
      public final static int rightClimberMotorPort = 11;

   }

   public static final class PneumaticsConstants {

      public final static int leftIntakeSolenoidPort = 1;
      public final static int rightIntakeSolenoidPort = 2;

      public final static int leftClimberSolenoidPort = 3;
      public final static int rightClimberSolenoidPort = 4;

   }

   public static final class DriveConstants {

  private final static double kEncoderCPR = 2048;
      //TODO: FIND 
  private final static double kGearReduction = 10.71;
  private final static double wheelDiameterMeters = 0.1524;
  private final static double kDistancePerWheelRevolutionMeters = wheelDiameterMeters * Math.PI;

  public static final double kMagMultiplier = (((kEncoderCPR * kGearReduction) / kDistancePerWheelRevolutionMeters));

//   private double encoderConstant = (1 / kGearReduction) * (1 / ENCODER_EDGES_PER_REV);
//   private double encoderConstantVelocity = (1 / kGearReduction) * (1 / ENCODER_EDGES_PER_REV) * 10;

   }

}
