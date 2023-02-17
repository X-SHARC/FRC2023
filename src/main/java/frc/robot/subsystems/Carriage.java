/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

 package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carriage extends SubsystemBase {
  WPI_TalonFX carriageMotor = new WPI_TalonFX(Constants.CARRIAGE_ID, "canavar");
  
  public Carriage() {
    carriageMotor.setInverted(false);
    carriageMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void intakeUp(){
    carriageMotor.set(ControlMode.PercentOutput, 0.3);
  }

  public void intakeDown(){
    carriageMotor.set(ControlMode.PercentOutput, -0.3);
  }

  public void stop(){
    carriageMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 