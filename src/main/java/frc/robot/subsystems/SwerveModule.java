package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.lib.util.Gearbox;

public class SwerveModule {
  private String name;
  private TalonFX driveMotor;
  private TalonFX angleMotor;
  private CANCoder rotEncoder;
  Gearbox driveRatio = new Gearbox(6.75, 1);
  
  public PIDController rotPID = new PIDController(0.003, 0, 0);
  //0084888


  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1.899, 1.899, 5.23523);
  //private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(2.5145, 1.5143, 0.128523);
  private final PIDController drivePID = new PIDController(0.0031, 0, 0);

  private boolean driveEncoderInverted = false; 

  private boolean isDriveMotorInverted = false;

  public SwerveModule(String name, TalonFX driveMotor, TalonFX angleMotor, CANCoder rotEncoder, boolean driveEncoderInverted, boolean isDriveMotorInverted, double offset) {
    this.name = name;
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.rotEncoder = rotEncoder;

    this.driveMotor.setNeutralMode(NeutralMode.Brake);
    this.angleMotor.setNeutralMode(NeutralMode.Brake);

    this.driveEncoderInverted = driveEncoderInverted;
    this.isDriveMotorInverted = isDriveMotorInverted;

    driveMotor.setInverted(isDriveMotorInverted);
    rotPID.disableContinuousInput();
    rotEncoder.configMagnetOffset(offset);
  }

  public double getDegrees(){
    return rotEncoder.getAbsolutePosition();
  }


  public SwerveModulePosition getModulePosition(){
    return new SwerveModulePosition(getPosition(), getAngle());
  }

  public double getPosition(){
    return 
      driveMotor.getSelectedSensorPosition() * (Constants.Swerve.wheelCircumference / (6.86 * 2048.0));
  }

  public double getDriveMotorRate(){
    return 
    driveRatio.calculate(
      ((getDriveRawVelocity() * 10) / 2048.0) * Constants.Swerve.wheelCircumference
    );
  }

  public double getDriveRawVelocity(){
    return (driveEncoderInverted ? -1 : 1) * driveMotor.getSelectedSensorVelocity();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
     getDegrees()
    );
  }

  public void calibrate(String Name, boolean offsetCalibration, boolean driveCalibration){
    /*if(offsetCalibration){
      SmartDashboard.putNumber("Swerve Rot Debug" + Name +  "Encoder Value", getAngle().getDegrees());
      SmartDashboard.putNumber("Swerve Rot Debug" + Name + " PID Setpoint", rotPID.getSetpoint());
      SmartDashboard.putNumber("Swerve Rot Debug" + Name + " PID Error", rotPID.getPositionError());
    }
    if(driveCalibration){
      SmartDashboard.putNumber("Swerve Drive Debug " + name + " Vel Actual", getDriveMotorRate() );
      SmartDashboard.putNumber("Swerve Drive Debug" + Name + " PID Error", drivePID.getVelocityError());
      SmartDashboard.putNumber("Swerve Drive Debug " + name + " Vel Setpoint ", drivePID.getSetpoint() );
    }*/
  }

  public void resetRotationEncoder(){
    rotEncoder.setPosition(0);
  }
  
  public void resetDriveEncoder(){
    driveMotor.setSelectedSensorPosition(0);    
  }

  public void outputDistance(){
    SmartDashboard.putNumber("Swerve Distance " + name , getPosition());
  }

  public void resetBothEncoders(){
    resetDriveEncoder();
    resetRotationEncoder();
  }

  public void stopMotors(){
    driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    angleMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void setClosedLoop(SwerveModuleState desiredState){
    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);
    double desiredRotation = currentRotation.getDegrees() + rotationDelta.getDegrees();

    angleMotor.set(TalonFXControlMode.PercentOutput, 
        MathUtil.clamp(
          ( rotPID.calculate(
                currentRotation.getDegrees(),
                desiredRotation 
                )), 
            -1.0, 
            1.0)
    );
     
    double driveOutput = driveFeedforward.calculate(state.speedMetersPerSecond)/ RobotController.getBatteryVoltage();
    driveOutput += drivePID.calculate(getDriveMotorRate(), state.speedMetersPerSecond);
    driveMotor.set(TalonFXControlMode.PercentOutput, driveOutput);
    
  }

  // ! Open loop drive
  public void setDesiredState(SwerveModuleState desiredState) {
    
    if(Math.abs(desiredState.speedMetersPerSecond)<0.001){
      stopMotors();
      return;
    }

    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);
    double desiredRotation = currentRotation.getDegrees() + rotationDelta.getDegrees();

    double outputP = rotPID.calculate(
      currentRotation.getDegrees(),
      desiredRotation 
      );

    angleMotor.set(TalonFXControlMode.PercentOutput, 
        MathUtil.clamp( 
          outputP, 
            -1.0, 
            1.0)
    );

    SmartDashboard.putNumber("1- Current Rotation: ", currentRotation.getDegrees());
    SmartDashboard.putNumber("2- Desired Rotation", desiredRotation);

    driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / Constants.Swerve.kMaxSpeed);
  }

  public double getSet(){
    return rotPID.getSetpoint();
  }

}