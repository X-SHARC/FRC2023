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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carriage extends SubsystemBase {
  WPI_TalonFX carriageMotor = new WPI_TalonFX(3, "canavar");
  private DutyCycleEncoder encoder = new DutyCycleEncoder(2);
  
  private double kP = 0.016;
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

  /** Creates a new Carriage. */
  public Carriage() {
    encoder.setConnectedFrequencyThreshold(200);
    carriageMotor.configFactoryDefault();
    carriageMotor.setInverted(true);
    carriageMotor.setNeutralMode(NeutralMode.Brake);
    carriagePID.setTolerance(1.5);
  }

  public void resetEncoder(){
    encoder.reset();
  }

  public double getDegrees(){
    return Math.IEEEremainder((encoder.get() * 360. + offset), 360.);
  }
  
  public void setDegrees(double setpoint){
    double PIDOutput =  MathUtil.clamp(carriagePID.calculate(getDegrees(), setpoint), -0.6, 0.6);
    carriageMotor.set(ControlMode.PercentOutput, (PIDOutput * 1));
  } 

  public void intakeUp(){
    carriageMotor.set(ControlMode.PercentOutput, 0.4);
  }

  public void intakeDown(){
    carriageMotor.set(ControlMode.PercentOutput, -0.4);
  }

  public void stop(){
    carriageMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
