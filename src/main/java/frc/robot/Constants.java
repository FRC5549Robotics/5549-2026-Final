// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int DRIVE_CONTROLLER = 0;
  public static final int OPERATOR_CONTROLLER = 1;

  // "rio" is the default bus. Use "CANivoreName" if you have a CANivore.
  public static final CANBus kDriveCANBus = new CANBus("lil clanker");


  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2; 
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; 
  public static final double FRONT_LEFT_MAGNET_OFFSET = 0.109619;
  // -0.002197; 

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;  
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; 
  public static final double FRONT_RIGHT_MAGNET_OFFSET = 0.222412;
  // -0.547852; 

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8; 
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; 
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; 
  public static final double BACK_LEFT_MAGNET_OFFSET = 0.247559;
  // -0.314209; 

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6; 
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; 
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; 
  public static final double BACK_RIGHT_MAGNET_OFFSET = 0.149658;
  // -0.610840;  

  // Define the conventional order of our modules when putting them into arrays
  public static final int FRONT_LEFT =0;
  public static final int FRONT_RIGHT =1;
  public static final int REAR_LEFT =2;
  public static final int REAR_RIGHT =3;

  public static final boolean kFrontLeftDriveEncoderReversed = false;
  public static final boolean kFrontRightDriveEncoderReversed = false;
  public static final boolean kRearLeftDriveEncoderReversed = true;
  public static final boolean kRearRightDriveEncoderReversed = true;

  public static final boolean kFrontLeftTurningEncoderReversed = false;
  public static final boolean kFrontRightTurningEncoderReversed = false;
  public static final boolean kRearLeftTurningEncoderReversed = true;
  public static final boolean kRearRightTurningEncoderReversed = true;

  public static final double kWheelDiameterMeters = 0.1016; //0.098; // 0.09398; // 3.7 in

  // The drive encoder reports in RPM by default. Calculate the conversion factor
  // to make it report in meters per second.
  public static final double kDriveGearRatio = 6.03;
  public static final double kDriveConversionFactor = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

  public static final double kTurningGearRatio = 287/11;

  public static final double kTurnPositionConversionFactor = 12.8;

  public static final double kMaxSpeedMetersPerSecond = 5;
  // Units are meters.
  // Distance between centers of right and left wheels on robot
  public static final double kTrackWidth = 0.51435;
  
  // Distance between front and back wheels on robot
  public static final double kWheelBase = 0.51435;

  // Units are meters per second
  public static final double kMaxTranslationalVelocity = 6784 / 60.0 *
  (1/kDriveGearRatio) *
  kWheelDiameterMeters * Math.PI;

  // Units are radians per second
  public static final double kMaxRotationalVelocity = kMaxTranslationalVelocity /
  Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);; //max 5.0

  //The locations f
  //*or the modules must be relative to the center of the robot. 
  // Positive x values represent moving toward the front of the robot 
  // Positive y values represent moving toward the left of the robot.
  public static final SwerveDriveKinematics kDriveKinematics =
  new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),   // front left
      new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),  // front right
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),  // rear left
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)  // rear right
      );


      
  public static final boolean kGyroReversed = false;

  public static final double kDriveP = 0.004; //Usually 0.05, 0.000000000000001?
  public static final double kDriveI = 0.0;
  public static final double kDriveD = 1; //Usually 0.0, 0.9;?
  public static final double kDriveFF = 0.1;
  public static final double SLEW_RATE_LIMITER = 3;

  public static final double kTurningP = 0.7; //Usually 0.05
  public static final double kTurningI = 0.0;
  public static final double kTurningD = 0.06;
  public static final double kAcceleration = 4;

  public static final int RESET_NAVX_BUTTON = 8;

  public static final Mode simMode = Mode.REAL;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
    

  //Shintake Constants
    // public static final int INTAKE_MOTOR_LEFT = 33;
    // public static final int INTAKE_MOTOR_RIGHT = 40;
    // public static final int SHOOTER_INTAKE_BUTTON = 6;
    // public static final double INTAKE_OUTTAKE_SPEED = 1;
    // public static final double OUTTAKE_L1_SPEED = 0.4;
    // public static final int SHINTAKE_BUTTON = 0;
    public static final int INTAKE_TRIGGER = 3;
    public static final int OUTTAKE_TRIGGER = 2;//RETURN TO OLD SETTING LATER

    //Intake Constants
    public static final int INTAKE_MOTOR = 17;
    public static final double INTAKE_SPEED = .65;
    public static final int INTAKE = 0;

   

  // //Limelight Constants
  //   public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0.0;
  //   public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1.0;
  //   public static final double X_SETPOINT_REEF_ALIGNMENT = 0.3;
  //   public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.01;    
  //   public static final double Y_SETPOINT_RIGHT_REEF_ALIGNMENT = 0.0;
  //   public static final double Y_SETPOINT_LEFT_REEF_ALIGNMENT = 0.0;
  //   public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.01;

  //Belt Constants
    public static final int BELT_MOTOR_ID = 18;

  //GroundIntake Constants
    public static final int PIVOT_MOTOR_ID = 18;
    public static final double PIVOT_DOWN_POSITION = 2;
    public static final double PIVOT_UP_POSITION = 0;
    public static final int GROUND_INTAKE_ID = 17;
    public static final double GROUND_PIVOT_SPEED = 0.15;
    public static final int PIVOT_ABS_ENC_DIO = 5;

  //Belt Constants
    public static final int Belt_MOTOR_ID = 14;
    //CanRange
    //public static final int CANRANGE_ID = 20;


    //Shooter constants
    public static final int LEFT_MOTOR_ID = 15;
    public static final int RIGHT_MOTOR_ID = 16;

    //HOOD constants
    public static final int HOOD_MOTOR_ID = 13;
    //CANdle
    public static final int CANDLE_ID = 19;
} 
