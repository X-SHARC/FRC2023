// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class Intake extends SubsystemBase {
  WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.INTAKE_ID);
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.setInverted(true);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void cubeidle(){
    intakeMotor.set(ControlMode.PercentOutput,-3.0/RobotContainer.pdh.getVoltage());
  }

  public void coneidle(){
    intakeMotor.set(ControlMode.PercentOutput,1.2/RobotContainer.pdh.getVoltage());
  }

  public void grabCube(){
    intakeMotor.set(ControlMode.PercentOutput, 0.65);
  }

  public void ejectCube(){
    intakeMotor.set(ControlMode.PercentOutput, -0.75);
  }

  public void grabCone(){
    intakeMotor.set(ControlMode.PercentOutput, -0.9);
  }

  public void ejectCone(){
    intakeMotor.set(ControlMode.PercentOutput, 0.9);
  }

  public void setPercent(double percent){
    intakeMotor.set(ControlMode.PercentOutput, 0.1);
  }

  public void stop(){
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
    RobotState.getInstance();
    RobotState.setIntakeIdle();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Current Game Piece:",RobotState.currentGamePiece.toString());

    // This method will be called once per scheduler run
  }
}