// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

      private final CANSparkMax driveMotor;
      private final VictorSPX turnMotor;
      
     // private final AnalogInput absoluteEncoder;

     private final AnalogInput absoluteEncoder;

     // private final AnalogEncoder absAnalogEncoder;

      private final RelativeEncoder driveEncoder;

      private final double absoluteEncoderOffsetRad;

      private final boolean absoluteEncoderReversed;

      private final PIDController m_turningPIDController  = new PIDController(1.5,0, 0);

     // private final Drivetrain m_swerves = new Drivetrain();

      /* 

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1.5,
          0,
          0,
          new TrapezoidProfile.Constraints(
              7, kModuleMaxAngularAcceleration));*/


public SwerveModule(int drivingCANId, int turningCANId,int absolutEncoderID, double Offset, boolean driveMotorReversed){

  driveMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
  turnMotor = new VictorSPX(turningCANId);

  absoluteEncoder = new AnalogInput(absolutEncoderID);
  //absAnalogEncoder = new AnalogEncoder(absolutEncoderID);

driveEncoder = driveMotor.getEncoder();

//absAnalogEncoder.setDistancePerRotation(Math.PI);

this.absoluteEncoderOffsetRad = Offset;
this.absoluteEncoderReversed = driveMotorReversed;



m_turningPIDController.enableContinuousInput(-Math.PI,Math.PI);
driveEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

driveEncoder.setPosition(0);


}
 
public double currentAngle(){
  double angle = (absoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 360);
  //angle *= 2.0 * Math.PI;

  angle += Math.toDegrees(absoluteEncoderOffsetRad);

  angle %= 360.0; // keep angle between 0 and 359 degrees
  if (angle < 0) {
    angle += 360.0; // adjust for negative angles
  }
  return Math.toRadians(angle);
}

public double limitedAngle(){
  double angle = (absoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 360);
  //angle *= 2.0 * Math.PI;

  angle += Math.toDegrees(absoluteEncoderOffsetRad);

  angle %= 360.0; // keep angle between 0 and 359 degrees
  if (angle < 0) {
    angle += 360.0; // adjust for negative angles
  }
  
  if (angle > 180) {
    angle -= 360; // adjust for angles greater than 180 degrees
  }
  
  return Math.toRadians(angle);
}



public double getDrivePosition() {
  return driveEncoder.getPosition();
}

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(currentAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(currentAngle()));
  }



 
  public void setDesiredState(SwerveModuleState desiredState) {


 SwerveModuleState state =
    //SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toDegrees(currentAngle())));
       Drivetrain.newOptimize(desiredState, new Rotation2d(currentAngle()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        state.speedMetersPerSecond /10;//ontroller.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
    m_turningPIDController.calculate(currentAngle(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.set(driveOutput);
    turnMotor.set(ControlMode.PercentOutput,turnOutput);
   SmartDashboard.putString("swerve"+ absoluteEncoder.getChannel()+ "og Angle", state.toString());
   // SmartDashboard.putString("swerve"+ absoluteEncoder.getChannel()+ "og Angle", desiredState.toString());
    SmartDashboard.putNumber("swerve"+ absoluteEncoder.getChannel() + "currentAngle", currentAngle());
    
  }
}
