package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {

  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  private static int currentPipeline = 0; // April Tags **default**

  public void setPipeline(int pipeline) {
    currentPipeline = pipeline;
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public int getCurrentPipeline() {
    return currentPipeline;
  }

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
    // double goalHeightInches = 24.0;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159265 / 180.0);


    double nodeATagsHeightInches;
    double aTagsDistanceFromLimelightToGoalInches;
    // double doubleSubstationATagsHeightInches;

    double rTapeHeightIn;
    double rTapeDistanceFromLimelightToGoalInches;

    if(currentPipeline == 0) {  //April Tags
        nodeATagsHeightInches = 18.25; 
        // doubleSubstationATagsHeightInches = 27.375;
        aTagsDistanceFromLimelightToGoalInches = (nodeATagsHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        return aTagsDistanceFromLimelightToGoalInches / 1.85; 
    } else if(currentPipeline == 1){ //Retro Tape
        rTapeHeightIn = 24.125;
        rTapeDistanceFromLimelightToGoalInches = (rTapeHeightIn - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
        return rTapeDistanceFromLimelightToGoalInches / 1.85;
    } 

    return 0.0;
  }

}
