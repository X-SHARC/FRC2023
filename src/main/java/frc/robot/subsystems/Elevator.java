package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  WPI_TalonFX elevatorMasterMotor = new WPI_TalonFX(19);
  WPI_TalonFX elevatorSlaveMotor = new WPI_TalonFX(32);

  public DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(1);

  private double kP = 0.031;
  private double kI = 0;
  private double kD = 0;

  private double kG = 0;
  private double kS = 0;
  private double kV = 0;
  private double kA = 0;

  public double distance;
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
    elevatorMasterMotor.setInverted(false);
    elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);
    elevatorSlaveMotor.setNeutralMode(NeutralMode.Brake);
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
     if (topLimitSwitch.get() == false){
     elevatorMasterMotor.set(ControlMode.PercentOutput, 0.1);
      } }

    public void elevatorDown(){
    if (bottomLimitSwitch.get() == false){
      elevatorMasterMotor.set(ControlMode.PercentOutput, -0.1);
    } }

    public void stop(){
      elevatorMasterMotor.set(ControlMode.PercentOutput, 0.0);
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Distance:", distance);
    SmartDashboard.putData(topLimitSwitch);
    SmartDashboard.putData(bottomLimitSwitch);
    // This method will be called once per scheduler run
    if(topLimitSwitch.get() == true||  bottomLimitSwitch.get() == true){
        stop();
    }

  }


}