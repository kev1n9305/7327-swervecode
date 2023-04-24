// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.PS4Controller;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PixyContants;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2Video;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {


  // private final PS4Controller m_controller = new PS4Controller(0);
  private final XboxController driverController = new XboxController(0);

 
  private final Drivetrain m_swerve = new Drivetrain();
  LimeLight mLight = new LimeLight();
  Pixy mPixy = new Pixy();
  Pixy2 pixy;
  int mode = 255;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  
  private LEDs leds;
  private LimeLight currentPipeline;

@Override
public void robotInit(){
  
  SPILink spiLink = new SPILink();
  pixy =Pixy2.createInstance(spiLink);
    pixy.init();
    pixy.setLED(255, 255 , 255);
    pixy.setCameraBrightness(50);

    leds = new LEDs(0);
    leds.setBurgundy();   
}

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(true);
    m_swerve.updateOdometry();
    leds.setGold();
  }

  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("Gyro angle", m_swerve.getGyroYaw());
    SmartDashboard.putNumber("distance", mLight.robotDistance());

    driveWithJoystick(true);
    GyroReset();

    mPixy.SendableData(pixy);
    mPixy.ledState(mPixy.toggleMode(mode, driverController.getAButton()), pixy);
    // mPixy.ledState(mPixy.toggleMode(mode, m_controller.getCrossButton()), pixy);

    if (driverController.getLeftBumper()){
      leds.setPurple();
      currentPipeline.setPipeline(0);
    } else if (driverController.getRightBumper()){
      leds.setYellow();
      currentPipeline.setPipeline(1);
    }

  }

  private void GyroReset(){
    m_swerve.resetHeading(driverController.getBButton());
  // m_swerve.resetHeading(m_controller.getCircleButton());
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
     var xSpeed =
        /* m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.09))
            * Drivetrain.kMaxSpeed; */
          m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftY(), 0.09))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
     var ySpeed =
        /* m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.09))
            * Drivetrain.kMaxSpeed; */
          m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftX(), 0.09))
            * Drivetrain.kMaxSpeed;


    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.

     var rot =
        /* -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.1))
            * Drivetrain.kMaxAngularSpeed;

            if(m_controller.getTriangleButton()){
             // xSpeed = -txOutput();
             xSpeed = mLight.txOutput();
              m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
            }
            if(m_controller.getCrossButton()){
              xSpeed = mPixy.xSpeedOutput(mPixy.G1Error(pixy));
              m_swerve.drive(-xSpeed, ySpeed, rot, fieldRelative);
            }
            else{
              m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
            } */
            -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getRightX(), 0.1))
            * Drivetrain.kMaxAngularSpeed;


            if(driverController.getYButton()){
             // xSpeed = -txOutput();
             xSpeed = mLight.txOutput();
              m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
            }
            if(driverController.getAButton()){
              xSpeed = mPixy.xSpeedOutput(mPixy.G1Error(pixy));
              m_swerve.drive(-xSpeed, ySpeed, rot, fieldRelative);
            }
            else{
              m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
            }
  }

}
