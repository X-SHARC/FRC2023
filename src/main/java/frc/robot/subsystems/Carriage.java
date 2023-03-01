// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Carriage extends SubsystemBase {
  private WPI_TalonFX carriageMotor = new WPI_TalonFX(Constants.CARRIAGE_ID, "canavar");
  public static DigitalInput limitSwitch = new DigitalInput(3);

  private double kP = 0.0031;
  private double kI = 0.0;
  private double kD = 0.0;

  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;

  PIDController carriagePID = new PIDController(kP, kI, kD);
  ArmFeedforward carriageFeedforward = new ArmFeedforward(kS, kG, kV, kA);

  double setpoint;
  private double PIDOutput = 0;
 // private double feedForwardOutput = 0;
  private double output = 0;

  double GearRatio1 = 1/152.4;
  // double GearRatio2 = 1/64;

  /** Creates a new Carriage. */
  public Carriage() {
    carriageMotor.setInverted(false);
    carriageMotor.setNeutralMode(NeutralMode.Brake);
    carriagePID.setTolerance(3);
  }

  public void resetCarriageEncoder(){
    carriageMotor.setSelectedSensorPosition(0);
  }

  public double getDegrees(){
    double angle;
    angle = carriageMotor.getSelectedSensorPosition() * GearRatio1;
    angle = (angle/2048.0) * 360;
    return angle;
  }

  public void setDegrees(double setpoint){
    PIDOutput = carriagePID.calculate(getDegrees(), setpoint);
  //  feedForwardOutput = carriageFeedforward.calculate(angle, setpoint, angle);
    output = (PIDOutput  /*+ feedForwardOutput */) / RobotController.getBatteryVoltage();
    if (limitSwitch.get() == false){
    carriageMotor.set(ControlMode.PercentOutput, PIDOutput * 1);
  }}

  public double getRadians(){
    double angle = carriageMotor.getSelectedSensorPosition();
    angle = (angle/2048.0) * 2 * Math.PI * GearRatio1  /* * GearRatio2 */;
    return angle;}

  public void intakeUp(){
    if (limitSwitch.get() == false){
    carriageMotor.set(ControlMode.PercentOutput, 0.4);
  }}

  public static boolean limiter(){
    return limitSwitch.get();
  }

  public void intakeDown(){
    carriageMotor.set(ControlMode.PercentOutput, -0.4);
  }

  public void stop(){
    carriageMotor.set(ControlMode.PercentOutput, 0.0);
  }


  @Override
  public void periodic() {
    RobotState.setCarriage(limitSwitch.get());
    SmartDashboard.putNumber("Carriage angle:", getDegrees());
    SmartDashboard.putData(limitSwitch);
    SmartDashboard.putNumber("Carriage PID ", PIDOutput);

    if (limitSwitch.get() == true){
      resetCarriageEncoder();
    }
    // This method will be called once per scheduler run
  }
}
