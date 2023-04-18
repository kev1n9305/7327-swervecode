package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    
        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
    
        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(17);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(18);
        // Distance between front and back wheels on robot
    
    
            /* 
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));*/
    
        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    
        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kRearLeftDrivingCanId = 2;
        public static final int kFrontRightDrivingCanId = 3;
        public static final int kRearRightDrivingCanId = 4;
    
        public static final int kFrontLeftTurningCanId = 5;
        public static final int kRearLeftTurningCanId = 7;
        public static final int kFrontRightTurningCanId = 6;
        public static final int kRearRightTurningCanId = 8;
    
        public static final int KfrontRightabsolutEncoderID = 0 ;
        public static final int KfrontLeftabsolutEncoderID = 2 ;
        public static final int KbackRightabsolutEncoderID = 1 ;
        public static final int KbackLeftabsolutEncoderID = 3 ;
    
        
        public static final double KfrontRightOffSet =- Math.toRadians(42) ; // backleft s0
        public static final double KfrontLeftOffSet = -Math.toRadians(-10) ; // backrigt s2
        public static final double KbackRightOffSet = -Math.toRadians(35+180) ; // frontleft s1
        public static final double KbackLeftOffSet = -Math.toRadians(-113) ;// front right
    
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;
    
    
    
    
    
    
        public static final boolean kGyroReversed = false;
      }

      
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.67;
    public static final double kTurningMotorGearRatio = 1 / 18.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    
    public static final double kTurningEncoderRot2Rad = kDriveMotorGearRatio * 2 * Math.PI;

   // public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;



    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 6000 / 60;
   // public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

  }

  public static final class PixyContants{

    
public static final double speedRange = 0.2;
public static final int center = 158;
public static final int numObjects = 0;
public static final double maxSpeedPixy = .35;

  }
}
