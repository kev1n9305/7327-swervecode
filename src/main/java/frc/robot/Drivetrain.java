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
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants.DriveConstants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = 4*Math.PI; // 1/2 rotation per second
  


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


//AnalogGyro m = new AnalogGyro(5);


public Drivetrain() {
  new Thread(() -> {
      try {
          Thread.sleep(3000);
          m_gyro.reset();
      } catch (Exception e) {
      }
  }).start();
}


public double getHeading() {
  return Math.IEEEremainder(m_gyro.getAngle(), 360);
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
            m_gyro.zeroYaw();
        }


public void resetHeading( boolean isReset){
  if(isReset){
  zeroHeading();
  }
}
public Rotation2d getRotation3d() {
  return Rotation2d.fromDegrees(getHeading());
}

public double getGyroYaw(){
  return m_gyro.getYaw();
}


 
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
  //  Rotation2d fieldRelativeHeading = getFieldRelativeHeading();
  
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getYaw()))
               // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
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


  public static SwerveModuleState newOptimize(
          SwerveModuleState desiredState, Rotation2d currentAngle) {

        var delta = desiredState.angle.minus(currentAngle);

        double newSpeed = desiredState.speedMetersPerSecond;
        
        Rotation2d ogAngle = desiredState.angle;

        Rotation2d optimizedAngle;
        if (Math.abs(delta.getDegrees()) > 170) { //160 works well
          optimizedAngle=  desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0));
          newSpeed = -newSpeed;
      
          return new SwerveModuleState(
            newSpeed,
              optimizedAngle);
        }
        else {
         return new SwerveModuleState(desiredState.speedMetersPerSecond, ogAngle);
       }
      }                                                                                                                                           
    }