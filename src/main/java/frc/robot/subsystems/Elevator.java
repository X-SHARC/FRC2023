// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  WPI_TalonFX elevatorMasterMotor = new WPI_TalonFX(Constants.ELEVATOR_SLAVE_ID);
  WPI_TalonFX elevatorSlaveMotor = new WPI_TalonFX(Constants.ELEVATOR_MASTER_ID);

  DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(1);

  private double kP = 1;
  private double kI = 2;
  private double kD = 3;
  private double kG = 4;

  private double kS = 1;
  private double kV = 2;
  private double kA = 3;

  private double distance;
  private double feedForwardOutput;
  private double PIDOutput;
  
  double gearCircumference = 4.8514 * Math.PI;

  double error;
  double output;

  public PIDController elevatorPID = new PIDController(kP, kI, kD);
  ElevatorFeedforward feedForward = new ElevatorFeedforward(kS, kG, kV, kA);
  /** Creates a new Elevator. */

  public Elevator() {
    elevatorSlaveMotor.setInverted(false);
    elevatorMasterMotor.setInverted(true);
    elevatorSlaveMotor.follow(elevatorMasterMotor);
  }

  /*   if (SensorValue(topLimitSwitch) == true) {
      elevatorMasterMotor.set();
    } else {
      elevatorMasterMotor.set();
    }
    if (SensorValue(bottomLimitSwitch) == true) {
      elevatorMasterMotor.set();
    } else{
      elevatorMasterMotor.set();
    }
  }  
   public void SetMotorSpeed(double speed){
      if ()
    }   */ 

    
    public void resetEncoder(){
      elevatorMasterMotor.setSelectedSensorPosition(0);
    }

    public void setDistance(double setpoint){
      double distance = elevatorMasterMotor.getSelectedSensorPosition();
      distance = (distance/2048.0) * gearCircumference;

      elevatorPID.setTolerance(3);
      PIDOutput = elevatorPID.calculate(distance, setpoint);
      feedForwardOutput = feedForward.calculate(60.0);
      output = (PIDOutput + feedForwardOutput) / RobotController.getBatteryVoltage();
      elevatorMasterMotor.set(ControlMode.PercentOutput, output);
    }


    public void elevatorUp(){
   //   if (topLimitSwitch.get() == false){
     elevatorMasterMotor.set(ControlMode.PercentOutput, 0.3);
      } //}

    public void elevatorDown(){
  //    if (bottomLimitSwitch.get() == false){
      elevatorMasterMotor.set(ControlMode.PercentOutput, -0.3);
    } //}

    public void stop(){
      elevatorMasterMotor.set(ControlMode.PercentOutput, 0.0);
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Value:", distance);
    // This method will be called once per scheduler run
  }


}
