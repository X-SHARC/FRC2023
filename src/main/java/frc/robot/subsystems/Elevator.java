package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private WPI_TalonFX elevatorMasterMotor = new WPI_TalonFX(19, "canavar");
  private WPI_TalonFX elevatorSlaveMotor = new WPI_TalonFX(32, "canavar");

  private DigitalInput topLimitSwitch = new DigitalInput(1);
  private DigitalInput bottomLimitSwitch = new DigitalInput(0);

  private double kP = 0.031;
  private double kI = 0;
  private double kD = 0;

  private double kG = 0;
  private double kS = 0;
  private double kV = 0;
  private double kA = 0;

  private double distance = 0;
  private double perpendicularDistance = 0;

  private double feedForwardOutput = 0;
  private double PIDOutput = 0;
  
  private double gearCircumference = 4.8514 * Math.PI;

  private double error = 0;
  private double output = 0;

  private PIDController elevatorPID = new PIDController(kP, kI, kD);
  private ElevatorFeedforward feedForward = new ElevatorFeedforward(kS, kG, kV, kA);
  

  public Elevator() {
    elevatorSlaveMotor.setInverted(false);
    elevatorMasterMotor.setInverted(false);
    elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);
    elevatorSlaveMotor.setNeutralMode(NeutralMode.Brake);
    elevatorSlaveMotor.follow(elevatorMasterMotor);
  }
  
    
    public void resetEncoder(){
      elevatorMasterMotor.setSelectedSensorPosition(0);
    }

    public double getDistance(){
      distance = elevatorMasterMotor.getSelectedSensorPosition();
      distance = (distance/2048.0) * gearCircumference;
      return distance;
    }

    public double getPerpendicularDistance(){
      return getDistance()*Math.sin(Units.degreesToRadians(55));
    }

    public void setDistance(double setpoint){
      double distance = getDistance();

      elevatorPID.setTolerance(3);
      PIDOutput = elevatorPID.calculate(distance, setpoint);
      feedForwardOutput = feedForward.calculate(60.0);
      output = (PIDOutput + feedForwardOutput) / RobotController.getBatteryVoltage();
      elevatorMasterMotor.set(ControlMode.PercentOutput, output);
    }


    public void elevatorUp(){
     if (topLimitSwitch.get() == false){
     elevatorMasterMotor.set(ControlMode.PercentOutput, 0.3);
      } }

    public void elevatorDown(){
    if (bottomLimitSwitch.get() == false){
      elevatorMasterMotor.set(ControlMode.PercentOutput, -0.3);
    } }

    public void stop(){
      elevatorMasterMotor.set(ControlMode.PercentOutput, 0.0);
    }

  @Override
  public void periodic() {
    perpendicularDistance = getPerpendicularDistance();
    distance = getDistance();
    SmartDashboard.putNumber("Elevator Distance:", distance);
    SmartDashboard.putNumber("Elevator Perpendicular Distance:", distance);
    SmartDashboard.putData(topLimitSwitch);
    SmartDashboard.putData(bottomLimitSwitch);


    if (topLimitSwitch.get() == true) stop();
    else if(bottomLimitSwitch.get() == true){
      stop();
      resetEncoder();
    } 

  }


}
