package frc.robot.lib.util;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class LED {

    private Solenoid led;

    public LED(int SolenoidPort){
        //!check the pneumatics module type
        led = new Solenoid(PneumaticsModuleType.CTREPCM, SolenoidPort);
    }

    public void toggle() {
        if (led.get())
            led.set(false);
        else led.set(true);
    }

    public boolean get() {
        return led.get();
    }

    public void turnOn(){
        led.set(true);
    }

    public void turnOff(){
        led.set(false);
    }

    
}
