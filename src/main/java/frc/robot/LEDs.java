package frc.robot;

import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class LEDs extends Robot {

    private PWMSparkMax  G = new PWMSparkMax(3);
    private PWM pwm, pwm2, pwm3 ; 

    private AddressableLED LedG;
    private AddressableLEDBuffer m_ledBuffer;
    @Override public void robotInit(){

        pwm = new PWM(4);

        pwm.setDisabled();
      }
      
@Override public void teleopPeriodic(){

  //  G.set(.5);

    pwm.setRaw(0);

   
}
}
    
    

