package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.ElevatorLevel;
import frc.robot.RobotState.SwerveState;
import frc.robot.commands.Elevator.ElevatorHome;

public class Elevator extends SubsystemBase {
  private WPI_TalonFX elevatorMasterMotor = new WPI_TalonFX(32, "canavar");
  private WPI_TalonFX elevatorSlaveMotor = new WPI_TalonFX(19, "canavar");
  private PowerDistribution pdh = new PowerDistribution();

  SlewRateLimiter limiter = new SlewRateLimiter(0.05);

  public DigitalInput topLimitSwitch = new DigitalInput(1);
  public DigitalInput bottomLimitSwitch = new DigitalInput(0);

  private double kP = 0.045;
  private double kI = 0.0;
  private double kD = 0.0;

  private double kG = 0.0;
  private double kS = 0.0;
  private double kV = 0.0;
  private double kA = 0.0; 

  private double distance = 0;
  private double perpendicularDistance = 0;

  // private double feedForwardOutput = 0;
  private double PIDOutput = 0;
  
  private double gearCircumference = Units.inchesToMeters(1.91) * Math.PI * 100;

  // private double error = 0;
  private double output = 0;

  private PIDController elevatorPID = new PIDController(kP, kI, kD);
  private ElevatorFeedforward feedForward = new ElevatorFeedforward(kS, kG, kV, kA);
  

  public Elevator() {

    RobotState.setElevating(false);

    elevatorMasterMotor.clearStickyFaults();

    elevatorSlaveMotor.setInverted(false);
    elevatorMasterMotor.setInverted(false);
    elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);
    elevatorSlaveMotor.setNeutralMode(NeutralMode.Brake);
    elevatorSlaveMotor.follow(elevatorMasterMotor);
    elevatorPID.setTolerance(1);
  }

    
    public void resetEncoder(){
      elevatorMasterMotor.setSelectedSensorPosition(0);
      RobotState.setElevating(false);
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

      
      PIDOutput = elevatorPID.calculate(distance, setpoint);
      //feedForwardOutput = feedForward.calculate(60.0);
      //output = (PIDOutput /*+ feedForwardOutput*/) / RobotController.getBatteryVoltage();
      if(topLimitSwitch.get() == true){
        elevatorMasterMotor.set(ControlMode.PercentOutput, PIDOutput);
        RobotState.setElevating(true);
      }
    }


    public void elevatorUp(){
     if (topLimitSwitch.get() == true){  // elektronik yanlış
        elevatorMasterMotor.set(ControlMode.PercentOutput, limiter.calculate(0.3));
        RobotState.setElevating(true);
      }
     }

    public void elevatorDown(){
    if (bottomLimitSwitch.get() == true){
      elevatorMasterMotor.set(ControlMode.PercentOutput, -limiter.calculate(0.3));
    } }

    public void stop(){
      elevatorMasterMotor.set(ControlMode.PercentOutput, 0.0);
      RobotState.setElevating(false);
    }

    public boolean getHome(){
      return !(bottomLimitSwitch.get());
    }

    public void elevatorHome(double speed){
      if (bottomLimitSwitch.get() == true){
        elevatorMasterMotor.set(ControlMode.PercentOutput, -limiter.calculate(speed));
      }
      else resetEncoder();

      RobotState.setElevating(true);
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
    RobotState.setElevatorLevel(getLevel());

    //SmartDashboard.putNumber("Current", pdh.getCurrent(19));
    perpendicularDistance = getPerpendicularDistance();
    distance = getDistance();
    SmartDashboard.putNumber("PIDOutput", PIDOutput);
    SmartDashboard.putNumber("Elevator Distance:", distance);
    SmartDashboard.putNumber("Elevator Perpendicular Distance:", perpendicularDistance);
    SmartDashboard.putData(topLimitSwitch);
    SmartDashboard.putData(bottomLimitSwitch);
        
    if (bottomLimitSwitch.get() == false){
      resetEncoder();
    }

    //WILL BE TESTED
    if(RobotState.getSwerveState() == SwerveState.MOVING){
      //Will this command work????
      //new ElevatorHome(this);
      stop();
    }

    } 

  }
