// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.DriveConstants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(.15, .15);
  private final Translation2d m_frontRightLocation = new Translation2d(.15, -.15);
  private final Translation2d m_backLeftLocation = new Translation2d(-.15, .15);
  private final Translation2d m_backRightLocation = new Translation2d(-.15, -.15);

  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.KfrontLeftabsolutEncoderID,
    DriveConstants.KfrontLeftOffSet,
    DriveConstants.kFrontLeftDriveEncoderReversed);

private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.KfrontRightabsolutEncoderID,
    DriveConstants.KfrontRightOffSet,
    DriveConstants.kFrontRightDriveEncoderReversed);

private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.KbackLeftabsolutEncoderID,
    DriveConstants.KbackLeftOffSet,
    DriveConstants.kBackLeftDriveEncoderReversed);

private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.KbackRightabsolutEncoderID,
    DriveConstants.KbackRightOffSet,
    DriveConstants.kBackRightDriveEncoderReversed);



private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);



public Drivetrain() {
  new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroHeading();
      } catch (Exception e) {
      }
  }).start();
}
  //private final AnalogGyro m_gyro = new AnalogGyro(5);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });
          public void zeroHeading() {
            m_gyro.reset();
        }
 // public Drivetrain() {
//    m_gyro.reset();
 // }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public static SwerveModuleState newerOptimize(SwerveModuleState desiredState, Rotation2d currentAngle) {

    double ogSpeed = desiredState.speedMetersPerSecond;

    var delta = desiredState.angle.minus(currentAngle);

    Rotation2d ogAngle = desiredState.angle;
    if (Math.abs(delta.getDegrees()) > 90.0) {
        Rotation2d desiredAngle = desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0));
        double desiredSpeed = -desiredState.speedMetersPerSecond;
        var angleCheck =   Math.abs(delta.getDegrees()) - Math.abs(ogAngle.getDegrees());
        if(angleCheck >=175){
         desiredSpeed = -desiredSpeed;
        }

        return new SwerveModuleState(desiredSpeed, desiredAngle);
    } else {
        return new SwerveModuleState(ogSpeed, ogAngle);
    }
}
  











  public static SwerveModuleState newOptimize(
          SwerveModuleState desiredState, Rotation2d currentAngle) {

            boolean isOptimized = false;
        var delta = desiredState.angle.minus(currentAngle);

        double ogSpeed = desiredState.speedMetersPerSecond;

        double newSpeed = desiredState.speedMetersPerSecond;
        
        Rotation2d ogAngle = desiredState.angle;

        Rotation2d optimizedAngle;
        if (Math.abs(delta.getDegrees()) > 160.0) { //160 works well
          optimizedAngle=  desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0));
          newSpeed = -newSpeed;
          isOptimized =true;
          return new SwerveModuleState(
            newSpeed,
              optimizedAngle);
        }
        else {
         return new SwerveModuleState(desiredState.speedMetersPerSecond, ogAngle);
       }
      }                                                                                                                                           
    
    
    


  public static SwerveModuleState junkOptimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double speedCheck= desiredState.speedMetersPerSecond;

    boolean wasOptimized = false;

    double unoptimizedAngle = desiredState.angle.getRadians();
    if (unoptimizedAngle < 0) {
      unoptimizedAngle += 2 * Math.PI;
    }

    double angleCheck = targetAngle;
    double distance = Math.abs(unoptimizedAngle - angleCheck);


    double delta = targetAngle - currentAngle.getDegrees();
    
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        wasOptimized = true;
    }
        if(distance >=170 && distance < 190 ){
          targetSpeed = -targetSpeed;
          wasOptimized = false;
        }

        //if(speedCheck > targetSpeed && angleCheck >180 || angleCheck < 180){
        //  targetSpeed= -targetSpeed;
       // }
    
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
  }
}