// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private WPI_TalonFX elevatorMasterMotor = new WPI_TalonFX(32, "canavar");
  private WPI_TalonFX elevatorSlaveMotor = new WPI_TalonFX(19, "canavar");
  SlewRateLimiter limiter = new SlewRateLimiter(0.05);
  public DigitalInput bottomLimitSwitch = new DigitalInput(0);

  private double kP = 0.045;
  private double kI = 0.0;
  private double kD = 0.0;

  private double distance = 0;
  private double perpendicularDistance = 0;

  private double PIDOutput = 0;
  
  private double gearCircumference = Units.inchesToMeters(1.91) * Math.PI * 100;

  private PIDController elevatorPID = new PIDController(kP, kI, kD);
  private double softLimit = (115*17.42*2048)/(gearCircumference*2);

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMasterMotor.configFactoryDefault();
    elevatorSlaveMotor.configFactoryDefault();
    
    elevatorSlaveMotor.setInverted(false);
    elevatorMasterMotor.setInverted(false);

    elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);
    elevatorSlaveMotor.setNeutralMode(NeutralMode.Brake);

    elevatorMasterMotor.setSafetyEnabled(false);
    elevatorMasterMotor.setSafetyEnabled(false);

    elevatorSlaveMotor.follow(elevatorMasterMotor);

    elevatorMasterMotor.configForwardSoftLimitEnable(true);
    elevatorMasterMotor.configForwardSoftLimitThreshold(softLimit);
  }
  
  public void resetEncoder(){
    elevatorMasterMotor.setSelectedSensorPosition(0);
  }

  
  public void elevatorUp(){
      elevatorMasterMotor.set(ControlMode.PercentOutput, limiter.calculate(0.45));
    
   }

  public void elevatorDown(){
  if (bottomLimitSwitch.get() == true){
    elevatorMasterMotor.set(ControlMode.PercentOutput, -limiter.calculate(0.45));
  }
 }

 public double getDistance(){
  distance = elevatorMasterMotor.getSelectedSensorPosition() / 17.42;
  distance = (distance/2048.0) * gearCircumference * 2;
  return distance;
}

public boolean isAtSetpoint(){
  return elevatorPID.atSetpoint();
}

public void setDistance(double setpoint){

  PIDOutput = MathUtil.clamp(
    elevatorPID.calculate(getDistance(), setpoint),
    -0.9,
    0.9
  );
  
  elevatorMasterMotor.set(ControlMode.PercentOutput, ((PIDOutput*12.)/RobotController.getBatteryVoltage()));
}

public boolean getHome(){
  return !(bottomLimitSwitch.get());
}

public void elevatorHome(double speed){
  if (bottomLimitSwitch.get() == true){
    elevatorMasterMotor.set(ControlMode.PercentOutput, -limiter.calculate(speed));
  }
  else{
    resetEncoder();
    stop();
  } 
    
}
  public void stop(){
    elevatorMasterMotor.set(ControlMode.PercentOutput, 0.0);
  }
      

@Override
public void periodic() {
  if (bottomLimitSwitch.get() == false){
    stop();
  }
  } 

}