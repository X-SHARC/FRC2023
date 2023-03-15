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
  public static DigitalInput minimumSwitch = new DigitalInput(3);
  //ilk değer: 0.01735
  private double kP = 0.016;
  //private double kP = 0.0;
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
 // private double feedForwardOutput = 0;
  private double output = 0;

  double GearRatio1 = 1/121.905;
  // double GearRatio2 = 1/64;
  private double offset = 105.2;

  private double softLimit = (115 * 2048)/(360 * GearRatio1);

  /** Creates a new Carriage. */
  public Carriage() {
    encoder.setConnectedFrequencyThreshold(200);
    carriageMotor.configFactoryDefault();
    carriageMotor.setInverted(true);
    carriageMotor.setNeutralMode(NeutralMode.Brake);
    carriagePID.setTolerance(1.5);


    //carriageMotor.configReverseSoftLimitEnable(true);
    //carriageMotor.configReverseSoftLimitThreshold(softLimit);
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
    //return encoder.getAbsolutePosition();
  }

  public void resetEncoder(){
    encoder.reset();
  }
  

  public void setDegrees(double setpoint){
    double PIDOutput =  MathUtil.clamp(carriagePID.calculate(getDegrees(), setpoint), -0.6, 0.6);
    //double feedForwardOutput = carriageFeedforward.calculate(setpoint,);
  //  output = (PIDOutput  /*+ feedForwardOutput */) / RobotController.getBatteryVoltage();
   // if (minimumSwitch.get() == false){
    carriageMotor.set(ControlMode.PercentOutput, (PIDOutput * 1));
  } //}

  public double getRadians(){
    angle = carriageMotor.getSelectedSensorPosition();
    angle = (angle/2048.0) * 2 * Math.PI * GearRatio1;
    return angle;}

  public void intakeUp(){
    //if (minimumSwitch.get() == false){
    carriageMotor.set(ControlMode.PercentOutput, 0.4);
   // }
  }

  /* public boolean limiter(){
    return minimumSwitch.get();
  }
  */

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
    if(minimumSwitch.get() == false){
      carriageMotor.set(ControlMode.PercentOutput, speed);
    }
    else{
      resetCarriageEncoder();
    }
    RobotState.setElevating(true);
  }

  public boolean getCarriageHome(){
    return (minimumSwitch.get());
  }

  public boolean isAlive(){
    return encoder.isConnected();
  }


  @Override
  public void periodic() {

    if(isAlive() && RobotState.getCarriage() == RobotState.CarriageState.AUTO){
      if(setpoint>=3 && setpoint<=115){
        setDegrees(setpoint);
      } 
    }
    else if(RobotState.getCarriage() == RobotState.CarriageState.MANUAL){

    }
    else{
      stop();
    }

    RobotState.setCarriage(minimumSwitch.get());
    SmartDashboard.putBoolean("Carriage Is Alive", isAlive());
    SmartDashboard.putNumber("Carriage Frequency", encoder.getFrequency());
    SmartDashboard.putNumber("Carriage Angle", getDegrees());
    SmartDashboard.putBoolean("Carriage Limit Switch", minimumSwitch.get());
    SmartDashboard.putNumber("Carriage PID Position Error", carriagePID.getPositionError());
    SmartDashboard.putNumber("Carriage Setpoint ", carriagePID.getSetpoint());

    /* 
    if (minimumSwitch.get() == true){
      stop();
      resetCarriageEncoder();
    }
    */

  }

}
