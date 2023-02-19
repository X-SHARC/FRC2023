// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Intake extends SubsystemBase {
  WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.INTAKE_ID);
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.setInverted(false);
  }

  public void grabCube(){
    intakeMotor.set(ControlMode.PercentOutput, 0.5);
    RobotState.getInstance();
    RobotState.setIntaking();

  }

  public void grabCone(){
    intakeMotor.set(ControlMode.PercentOutput, -0.5);
    RobotState.getInstance();
    RobotState.setEjecting();
  }

  public void stop(){
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
    RobotState.getInstance();
    RobotState.setIntakeIdle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}