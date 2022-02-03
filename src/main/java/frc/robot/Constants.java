// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // O I - C O N S T A N T S //
    public static int driveControllerPort = 0;
    public static int techControllerPort = 1; 

    public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
	 public final static Gains kGains_Turning = new Gains( 0.10, 0.0,  0.0, 0.0,            200,  1.00 );
    public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);
	 public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/20660.0,  400,  1.00 );

    public final static double kNeutralDeadband = 0.001;
    public final static int kTimeoutMs = 30;
    public final static int kSensorUnitsPerRotation = 2048;


    public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;

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

    public final static Spark turretMotor = new Spark(4);
    
    public final static int frontLeftDrivePort = 1;
    public final static int backLeftDrivePort = 2;
    public final static int frontRightDrivePort = 3;
    public final static int backRightDrivePort = 4;
    public final static int collectorMotorPort = 5;
    public final static int conveyorMotorPort = 6;

 }

}
