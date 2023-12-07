package frc.robot.lib.util;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SRXMagEncoder {

    private WPI_TalonSRX sensorTalon;
    private double cpr = 4096.;
    private double distancePerRot;

    public SRXMagEncoder(WPI_TalonSRX talonSRX){
        this.sensorTalon = talonSRX;
    }

    public void setDistancePerRotation(double meters){
        this.distancePerRot = meters;
    }

    public double getPosition(){
        return sensorTalon.getSelectedSensorPosition()/cpr * distancePerRot;
    }

    public void reset(){
        sensorTalon.setSelectedSensorPosition(0);
    }
}
