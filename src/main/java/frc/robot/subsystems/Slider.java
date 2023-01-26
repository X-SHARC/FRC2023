// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slider extends SubsystemBase {
  WPI_TalonFX sliderMotor = new WPI_TalonFX(Constants.SLIDER_ID);
  
  /** Creates a new Slider. */
  public Slider() {
    sliderMotor.setInverted(false);
  }

  public void sliderForward(){
    sliderMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void sliderBackwards(){
    sliderMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void stop(){
    sliderMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
