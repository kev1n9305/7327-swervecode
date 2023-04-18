package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    
    public double txOutput() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    
        final PIDController tXPidController = new PIDController(.125, 0.0, 0);
        tXPidController.enableContinuousInput(-27, 27);
        double output = tXPidController.calculate(tx, 0);
    
        return output;
    }
    
    
public double robotDistance(){

    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
   double targetOffsetAngle_Vertical = ty;
  
  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 0.0;
  
  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 21.5;
  
  // distance from the target to the floor
  double goalHeightInches = 24.0;
  
  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  
  //calculate distance
  double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
  
  
    return distanceFromLimelightToGoalInches / 1.85;
  }
}
