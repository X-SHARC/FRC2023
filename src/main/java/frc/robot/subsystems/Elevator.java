// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  WPI_TalonFX elevatorMasterMotor = new WPI_TalonFX(Constants.ELEVATOR_SLAVE_ID);
  WPI_TalonFX elevatorSlaveMotor = new WPI_TalonFX(Constants.ELEVATOR_MASTER_ID);

  DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(1);

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorSlaveMotor.setInverted(false);
    elevatorMasterMotor.setInverted(true);
    elevatorSlaveMotor.follow(elevatorSlaveMotor);
  }

double gearCircumference = 4.8514 * Math.PI;

  private double kP = 0.11533;
  private double kI = 0.0;
  private double kD = 0.0;

  private double kS = 0.74791;
  private double kV = 0.11447;
  private double kA = 0.0063222;
  private double kG = 0.0063222;

  ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  private double PIDOutput;
  private double feedForwardOutput;
  private double distance;

  public PIDController elevatorPID = new PIDController(kP, kI, kD);  

  double error;
  double output;


  public void resetEncoder() {
    distance = 0.0; 
    // encoder in raw datasını donüş sayısına dönüştürme
  // dönüş sayısı distance a 
  }

  //merhaba harun hocam saygılar

  public void setDistance(double setpoint) {
    double distance = elevatorMasterMotor.getSelectedSensorPosition();
    distance = (distance/2048.0) * gearCircumference;
    elevatorPID.setTolerance(100);
    PIDOutput = elevatorPID.calculate(60., setpoint);
    feedForwardOutput = feedforward.calculate(60.);

    //output = (PIDOutput + feedForwardOutput) / RobotController.getBatteryVoltage();
    output = (PIDOutput + feedForwardOutput);
    elevatorMasterMotor.set(ControlMode.PercentOutput,output);
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

   public void elevatorUp(){
     elevatorMasterMotor.set(ControlMode.PercentOutput, 0.5);
      }

    public void elevatorDown(){
      elevatorMasterMotor.set(ControlMode.PercentOutput, -0.5);
    }

    public void stop(){
      elevatorMasterMotor.set(ControlMode.PercentOutput, 0.0);
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder:", distance);
    // This method will be called once per scheduler run
  }
}

