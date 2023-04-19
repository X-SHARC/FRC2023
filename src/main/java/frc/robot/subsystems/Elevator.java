package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.ElevatorLevel;

public class Elevator extends SubsystemBase {
  private WPI_TalonFX elevatorMasterMotor = new WPI_TalonFX(32, "canavar");
  private WPI_TalonFX elevatorSlaveMotor = new WPI_TalonFX(19, "canavar");

  SlewRateLimiter limiter = new SlewRateLimiter(0.05);

  public DigitalInput bottomLimitSwitch = new DigitalInput(0);

  private double kP = 0.045;
  private double kI = 0.0;
  private double kD = 0.0;

  private double distance = 0;
  private double perpendicularDistance = 0;

  private double PIDOutput = 0;
  
  private double gearCircumference = Units.inchesToMeters(1.91) * Math.PI * 100;

  private PIDController elevatorPID = new PIDController(kP, kI, kD);

  private double softLimit = (115*17.42*2048)/(gearCircumference*2);

  public Elevator() {
    elevatorMasterMotor.configFactoryDefault();
    elevatorSlaveMotor.configFactoryDefault();
    RobotState.setElevated(false);

    //elevatorMasterMotor.clearStickyFaults();
    //elevatorSlaveMotor.clearStickyFaults();

    elevatorSlaveMotor.setInverted(false);
    elevatorMasterMotor.setInverted(false);

    elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);
    elevatorSlaveMotor.setNeutralMode(NeutralMode.Brake);

    elevatorMasterMotor.setSafetyEnabled(false);
    elevatorMasterMotor.setSafetyEnabled(false);

    elevatorSlaveMotor.follow(elevatorMasterMotor);
    elevatorPID.setTolerance(1);

    elevatorMasterMotor.configForwardSoftLimitEnable(true);
    elevatorMasterMotor.configForwardSoftLimitThreshold(softLimit);
  }

    public void resetEncoder(){
      elevatorMasterMotor.setSelectedSensorPosition(0);
    }

    public double getError(){
      return elevatorPID.getPositionError();
    }

    public double getDistance(){
      distance = elevatorMasterMotor.getSelectedSensorPosition() / 17.42;
      distance = (distance/2048.0) * gearCircumference * 2;
      return distance;
    }

    public double getPerpendicularDistance(){
      return getDistance()*Math.sin(Units.degreesToRadians(55));
    }

    public boolean isAtSetpoint(){
      return elevatorPID.atSetpoint();
    }

    public void setDistance(double setpoint){
      double distance = getDistance();

      PIDOutput = MathUtil.clamp(
          elevatorPID.calculate(distance, setpoint),
          -1,
          1
        );
        elevatorMasterMotor.set(ControlMode.PercentOutput, ((PIDOutput*12.)/RobotController.getBatteryVoltage()));
        
        //if(Math.abs(distance-setpoint)<3) stop();
    }

    public void elevatorUp(){
        elevatorMasterMotor.set(ControlMode.PercentOutput, limiter.calculate(0.45));
      
     }

    public void set(double speed){
      if(speed<0){
        if (bottomLimitSwitch.get() == true){
          elevatorMasterMotor.set(ControlMode.PercentOutput, -limiter.calculate(Math.abs(speed)));
        }
      }
      else if(speed>0){
          elevatorMasterMotor.set(ControlMode.PercentOutput, limiter.calculate(Math.abs(speed)));
        
      }
      else stop();
    }

    public void elevatorDown(){
    if (bottomLimitSwitch.get() == true ){
      elevatorMasterMotor.set(ControlMode.PercentOutput, -limiter.calculate(0.45));
    }
   }

    public void stop(){
      elevatorMasterMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public boolean getHome(){
      return !(bottomLimitSwitch.get());
    }

    public void elevatorHome(double speed){
      if (bottomLimitSwitch.get() == true){
        elevatorMasterMotor.set(ControlMode.PercentOutput, -limiter.calculate(speed));
      }
      else{
        resetEncoder();
        stop();
      } 
        
    }

    private void saveFalcons(){
        //Check the stalling ampers and type; eğer encoder değişmiyorsa ve amper yüksek geliyorsa durdur
    }

    //The conditional distance values can be arranged according to the need.
    private ElevatorLevel getLevel(){
      if(getDistance()>25 && getDistance() <70){
        return ElevatorLevel.MIDROW;
      }
      else if(getDistance()>70){
        return ElevatorLevel.TOPROW;
      }
      else return ElevatorLevel.ZERO;
      
    }

  @Override
  public void periodic() {
    double distance = getDistance();
    /*SmartDashboard.putNumber("PIDOutput", PIDOutput);
    SmartDashboard.putNumber("Elevator PID Error", elevatorPID.getPositionError());
    SmartDashboard.putNumber("Elevator PID Setpoint", elevatorPID.getSetpoint());
    
    SmartDashboard.putData("Bottom Limit Switch", bottomLimitSwitch);*/
    SmartDashboard.putNumber("Elevator Distance:", distance);

    if(bottomLimitSwitch.get() == false){
      resetEncoder();
      RobotState.setElevated(false);
    }
    else if(distance>= 28) RobotState.setElevated(true);
    else RobotState.setElevated(false);

    } 

  }
