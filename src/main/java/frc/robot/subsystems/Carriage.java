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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Carriage extends SubsystemBase {
  private WPI_TalonFX carriageMotor = new WPI_TalonFX(Constants.CARRIAGE_ID, "canavar");
  public static DigitalInput CarriageLimitSwitch = new DigitalInput(3);
  //ilk değer: 0.01735
  private double kP = 0.02731;
  private double kI = 0.0;
  private double kD = 0.0;

  private double kS = 0.0;
  private double kG = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;

  PIDController carriagePID = new PIDController(kP, kI, kD);
  ArmFeedforward carriageFeedforward = new ArmFeedforward(kS, kG, kV, kA);

  private double angle = 0;

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
    carriagePID.setTolerance(1.5);
  }

  public void resetCarriageEncoder(){
    carriageMotor.setSelectedSensorPosition(0);
  }

  public double getDegrees(){
    angle = carriageMotor.getSelectedSensorPosition() * GearRatio1;
    angle = (angle/2048.0) * 360;
    return angle;
  }

  public void setDegrees(double setpoint){
    PIDOutput =  MathUtil.clamp(carriagePID.calculate(angle, setpoint), -0.87, 0.87);
  //  feedForwardOutput = carriageFeedforward.calculate(angle, setpoint, angle);
  //  output = (PIDOutput  /*+ feedForwardOutput */) / RobotController.getBatteryVoltage();
   // if (CarriageLimitSwitch.get() == false){
    carriageMotor.set(ControlMode.PercentOutput, (PIDOutput * 1));
  } //}

  public double getRadians(){
    angle = carriageMotor.getSelectedSensorPosition();
    angle = (angle/2048.0) * 2 * Math.PI * GearRatio1  /* * GearRatio2 */;
    return angle;}

  public void intakeUp(){
    if (CarriageLimitSwitch.get() == false){
    carriageMotor.set(ControlMode.PercentOutput, 0.4);
  }}

  public boolean limiter(){
    return CarriageLimitSwitch.get();
  }

  public void intakeDown(){
    carriageMotor.set(ControlMode.PercentOutput, -0.4);
  }

  public void stop(){
    carriageMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean isAtSetpoint(){
    return carriagePID.atSetpoint();
  }

  public void carriageHome(double speed){
    if(CarriageLimitSwitch.get() == false){
      carriageMotor.set(ControlMode.PercentOutput, speed);
    }
    else{
      resetCarriageEncoder();
    }
  }

  public boolean getElevatorHome(){
    return (CarriageLimitSwitch.get());
  }


  @Override
  public void periodic() {
    RobotState.setCarriage(CarriageLimitSwitch.get());
    SmartDashboard.putNumber("Carriage angle:", getDegrees());
    SmartDashboard.putData("Carriage Limit Switch", CarriageLimitSwitch);
    SmartDashboard.putNumber("Carriage PID ", PIDOutput);

    if (CarriageLimitSwitch.get() == true){
      stop();
      resetCarriageEncoder();
    }


    // This method will be called once per scheduler run
  }
}
