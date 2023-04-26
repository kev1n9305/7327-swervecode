package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PixyContants;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

public class Pixy {

    public void SendableData(Pixy2 pixy){
            
        pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
        List<Pixy2CCC.Block> blocksList = pixy.getCCC().getBlockCache();

        

        if (!blocksList.isEmpty()) {
            int numObjectsX = blocksList.get(0).getX();
            int error = PixyContants.center - numObjectsX;

            // debug info
            Pixy2CCC.Block block = blocksList.get(0);
            int x = block.getX();
            int y = block.getY();
            int width = block.getWidth();
            int height = block.getHeight();


            SmartDashboard.putNumber("off center error", error);
            SmartDashboard.putNumber("number of objects in the frame ", numObjectsX);
            SmartDashboard.putNumber(", x:", x);
            SmartDashboard.putNumber(", y: ", y);
            SmartDashboard.putNumber(", width: ", width);
            SmartDashboard.putNumber(", height: ", height);
        }
        
    }


    public int G1Error(Pixy2 pixy) {
        pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
        List<Pixy2CCC.Block> blocksList = pixy.getCCC().getBlockCache();

        if (!blocksList.isEmpty()) {
            int numObjectsX = blocksList.get(0).getX();
            return PixyContants.center - numObjectsX;
        } else {
            return 0;
        }
    }

    public int getNumObjectsG1(Pixy2 pixy) {
        pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
        List<Pixy2CCC.Block> blocksList = pixy.getCCC().getBlockCache();

        if (!blocksList.isEmpty()) {
            return blocksList.size();
        } else {
            return 0;
        }
    }

    public double xSpeedOutput(int error){

        PIDController pixyPidController = new PIDController(.05, 0.0, 0);

        pixyPidController.enableContinuousInput(-156,156 );
        double output = pixyPidController.calculate(error, 0);

        return output*.35;
    }

    public int toggleMode(int mode, boolean controller) {
        if (controller) {
            mode = (mode + 1) % 2; // toggle mode between 0 and 1
        } else {
            mode = (mode + 1) % 2 + 1; // toggle mode between 1 and 2
        }
        return mode == 1 ? 0 : 255; // return 0 if mode is 1, otherwise return 255
    }

    public int ledState(int mode, Pixy2 pixy){

    return pixy.setLamp((byte) mode, (byte) 255);
    }
}