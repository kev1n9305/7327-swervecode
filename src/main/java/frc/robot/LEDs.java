package frc.robot;

// import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.PWM;


public class LEDs {
    private PWM redOutput;
    private PWM greenOutput;
    private PWM blueOutput;

    public LEDs(int usbPort) {
     // PortForwarder.add(usbPort, null, usbPort);

      redOutput = new PWM(usbPort); // red pin
      greenOutput = new PWM(usbPort); // green pin
      blueOutput = new PWM(usbPort); // blue pin

      setBurgundy(); 
    }



    public void setBurgundy() { // default
      redOutput.setRaw(119);   // R: 119, G: 0, B: 28
      greenOutput.setRaw(0);
      blueOutput.setRaw(28);
      System.out.println("Burgundy!");
    }
    
    public void setGold() { // autunomous?
      redOutput.setRaw(175);   // R: 175, G: 155, B: 35
      greenOutput.setRaw(155);
      blueOutput.setRaw(35);
      System.out.println("Gold!");
    }

    public void setPurple() {
      redOutput.setRaw(127);   // R: 127, G: 0, B: 255 **(cubes/Pipe0)**
      greenOutput.setRaw(0);
      blueOutput.setRaw(255);
      System.out.println("CUBES");
    }

    public void setYellow() {   // R: 255, G: 255, B: 0 **(cones/Pipe1)**
      redOutput.setRaw(255);
      greenOutput.setRaw(255);
      blueOutput.setRaw(0);
      System.out.println("CONES");
    }

    
    
}
