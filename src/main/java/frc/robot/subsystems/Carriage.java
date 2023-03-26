// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Carriage extends SubsystemBase {
  private WPI_TalonFX carriageMotor = new WPI_TalonFX(Constants.CARRIAGE_ID, "canavar");
  private DutyCycleEncoder encoder = new DutyCycleEncoder(2);
  
  private double kP = 0.01877;
  private double kI = 0.0;
  private double kD = 0.0;

  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;

  PIDController carriagePID = new PIDController(kP, kI, kD);
  ArmFeedforward carriageFeedforward = new ArmFeedforward(kS, kG, kV, kA);

  private double angle = 0;

  double setpoint = 5;

  double GearRatio1 = 1/121.905;
  private double offset = 105.2;

  public Carriage() {
    encoder.setConnectedFrequencyThreshold(200);
    carriageMotor.configFactoryDefault();
    carriageMotor.setInverted(true);
    carriageMotor.setNeutralMode(NeutralMode.Brake);
    carriagePID.setTolerance(1.5);
  }

  public void resetCarriageEncoder(){
    carriageMotor.setSelectedSensorPosition(0);
  }

  public double getIntegratedDegrees(){
    angle = carriageMotor.getSelectedSensorPosition() * GearRatio1;
    angle = (angle/2048.0) * 360;
    return angle;
  }

  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
  }

  public double getDegrees(){
    return Math.IEEEremainder((encoder.get() * 360. + offset), 360.);
  }

  public void resetEncoder(){
    encoder.reset();
  }
  
  public void setDegrees(double setpoint){
    
    if(isAlive() && RobotState.getCarriage() == RobotState.CarriageState.AUTO){ 
        double currAngle = getDegrees();
        if(currAngle<=115){
          double PIDOutput =  MathUtil.clamp(carriagePID.calculate(currAngle, setpoint), -0.6, 0.6);
          carriageMotor.set(ControlMode.PercentOutput, (PIDOutput * 1));
        } 
        else if(currAngle>115) carriageMotor.set(ControlMode.PercentOutput, -0.1);
        else stop();
      
    }
    else stop();

    }
    

  public double getRadians(){
    angle = carriageMotor.getSelectedSensorPosition();
    angle = (angle/2048.0) * 2 * Math.PI * GearRatio1;
    return angle;}

  public void intakeUp(){
    carriageMotor.set(ControlMode.PercentOutput, 0.6);
  }

  public void intakeDown(){
    carriageMotor.set(ControlMode.PercentOutput, -0.6);
  }

  public void stop(){
    carriageMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean isAtSetpoint(){
    return carriagePID.atSetpoint();
  }

  public void carriageHome(double speed){
    setSetpoint(5);
  }

  public boolean getCarriageHome(){
    return (getDegrees()<=5);
  }

  public boolean isAlive(){
    return encoder.isConnected();
  }

  @Override
  public void periodic() {
    boolean isAlive = isAlive();
    RobotState.setCarriageEncoder(isAlive);
    if(getDegrees()>=100 || getDegrees()<=5){
      stop();
      RobotState.setCarriageLimit(true);
    }
    else RobotState.setCarriageLimit(false);

    /*SmartDashboard.putBoolean("Carriage Is Alive", isAlive());
    SmartDashboard.putNumber("Carriage PID Position Error", carriagePID.getPositionError());
    SmartDashboard.putNumber("Carriage Setpoint ", carriagePID.getSetpoint());*/
    SmartDashboard.putNumber("Carriage Angle", getDegrees());
  }

}
