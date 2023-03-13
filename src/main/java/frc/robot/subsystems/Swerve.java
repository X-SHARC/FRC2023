// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.SwerveState;

public class Swerve extends SubsystemBase {

  /*
  TODO realtime PID tuning - look into docs
  TODO look into feedforward tuning

  */

  private boolean isCalibrating;
  private boolean offsetCalibration = true;
  private boolean driveCalibration = false;
  private boolean rotCalibration = true;

  private Rotation2d fieldAngle = new Rotation2d();

  private final Field2d field2D = new Field2d();
  
  /**
   * TODO: These are example values and will need to be adjusted for your robot!
   * Modules are in the order of -
   * Front Left
   * Front Right
   * Back Left
   * Back Right
   * 
   * Positive x values represent moving toward the front of the robot whereas
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */

  //private final CKIMU gyro;
  private AHRS gyroAhrs = new AHRS();
  //TODO: set device number
  public Pigeon2 pigeon = new Pigeon2(23);


  // TODO: Update these CAN device IDs to match your TalonFX + CANCoder device IDs | Done
  // TODO: Update module offsets to match your CANCoder offsets | Done

  private double[] pidValues = {
    0.09698,
    0.09698,
    0.09698,
    0.09698
  };


  final boolean invertAllModules = true;
  private SwerveModule[] modules = new SwerveModule[] {

    new SwerveModule(
      "FL", 
      new TalonFX(17),
      new TalonFX(13),
      new CANCoder(4),
      false,
      false,
      new PIDController(pidValues[0], 0, 0),
      -298), //! Front Left

    new SwerveModule(
      "FR",
      new TalonFX(14),
      new TalonFX(18),
      new CANCoder(3),
      false,
      false,
      new PIDController(pidValues[1], 0, 0),-40), //! Front Right

    new SwerveModule(
      "RL",
      new TalonFX(11),
      new TalonFX(16),
      new CANCoder(2), 
      false,
      false,
      new PIDController(pidValues[2], 0, 0),-35), //! Back Left

    new SwerveModule(
      "RR",
      new TalonFX(10),
      new TalonFX(12),
      new CANCoder(1),
      false,
      false,
      new PIDController(pidValues[3], 0, 0),-11)  //! Back Right
  };


  SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
    modules[0].getModulePosition(),
    modules[1].getModulePosition(),
    modules[2].getModulePosition(),
    modules[3].getModulePosition()
  };

  public Swerve(boolean isCalibrating) {
    this.isCalibrating = isCalibrating;
    resetAllEncoders();
    
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyroAhrs.reset();
      } catch (Exception e) {
        //TODO: handle exception
      }
    }).start();
  
    SmartDashboard.putData("Field", field2D);
  }
  
  public Rotation2d getGyro(){
    return Rotation2d.fromDegrees(
        getGyroDouble()
        );
  }

  public double getNavxDouble(){
    return Math.IEEEremainder(gyroAhrs.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getGyroDouble(){
    return Math.IEEEremainder(pigeon.getYaw(), 360) * (Constants.kGyroReversed ? 1.0 : -1.0); 
  }

  public void resetPigeon(){
    pigeon.setYaw(0);
  }

  //can be used to align the charge station
  public double getPitch(){
    return pigeon.getPitch();
  }

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.Swerve.kinematics, gyroAhrs.getRotation2d(),
  swerveModulePositions);

  private Rotation2d teleopAngle = new Rotation2d(0);

  public void stopModules(){
    modules[0].stopMotors();
    modules[1].stopMotors();
    modules[2].stopMotors();
    modules[3].stopMotors();
  }
  

  public void resetAllEncoders(){
    resetPigeon();
    for (int i = modules.length-1; i >= 0; i--) {
      SwerveModule module = modules[i];
      //module.resetRotationEncoder();
      module.resetDriveEncoder();
    }
  }

  public void resetRotEncoders(){
    for (int i = modules.length-1; i >= 0; i--) {
      SwerveModule module = modules[i];
      //module.resetRotationEncoder();
      module.resetRotationEncoder();
    }
  }

  public double getAverageDistance(){
    double sum = 0;
    for (int i = modules.length-1; i >= 0; i--) {
      SwerveModule module = modules[i];
      sum += module.getPosition();
    }
    return sum / modules.length;
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      getGyro(),
      swerveModulePositions,
      pose
    );
    // resetAllEncoders();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if(xSpeed<0.1 && ySpeed<0.1 && rot<0.2) RobotState.setSwerve(SwerveState.REST);
    else RobotState.setSwerve(SwerveState.MOVING);

    SwerveModuleState[] states =
    Constants.Swerve.kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyro())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.kMaxSpeed);
    //setClosedLoopStates(states);
    
    for (int i = 0; i < states.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = states[i];
      module.setDesiredState(state);
    }

    
  }

  public void resetFieldOrientation() {
    resetFieldOrientation(new Rotation2d(getGyroDouble()));
  }
  public void resetFieldOrientation(Rotation2d angle) {
    this.fieldAngle = angle;
  }

  public void resetFieldOrientedTeleOp(){
    this.teleopAngle = getGyro();
  }

  public void manualGyroReset(){
    this.teleopAngle = new Rotation2d(0);
    this.fieldAngle = new Rotation2d(0);
    gyroAhrs.reset();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Swerve.kMaxSpeed);
    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);
  }

  public void setClosedLoopStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Swerve.kMaxSpeed);
    modules[0].setClosedLoop(desiredStates[0]);
    modules[1].setClosedLoop(desiredStates[1]);
    modules[2].setClosedLoop(desiredStates[2]);
    modules[3].setClosedLoop(desiredStates[3]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("swerve groAngle", getGyroDouble());
    SmartDashboard.putNumber("swerve field offset", fieldAngle.getDegrees());
    //SmartDashboard.putNumber("Pigeon Yaw:", getGyroDouble());

    
    // SmartDashboard.putNumber("0. SETPOINT", modules[0].drivePID.getSetpoint());
    // /SmartDashboard.putNumber("0. Velocity", modules[0].getDriveMotorRate());

    // SmartDashboard.putNumber("1. SETPOINT", modules[1].drivePID.getSetpoint());
    // SmartDashboard.putNumber("1. Velocity", modules[1].getDriveMotorRate());

    // SmartDashboard.putNumber("2. SETPOINT", modules[2].drivePID.getSetpoint());
    // SmartDashboard.putNumber("2. Velocity", modules[2].getDriveMotorRate());

    // SmartDashboard.putNumber("3. SETPOINT", modules[3].drivePID.getSetpoint());
    // SmartDashboard.putNumber("3. Velocity", modules[3].getDriveMotorRate());
    
    /*
    SmartDashboard.putNumber("1. modül", modules[1].getDriveMotorRate());
    SmartDashboard.putNumber("2. modül", modules[2].getDriveMotorRate());
    SmartDashboard.putNumber("3. modül", modules[3].getDriveMotorRate());
    SmartDashboard.putNumber("average Distance", getAverageDistance());
    */
    SmartDashboard.putNumber("Posex", getPose().getX());
    SmartDashboard.putNumber("Posey", getPose().getY());
    SmartDashboard.putNumber("Rot", getPose().getRotation().getRadians());

    /*SmartDashboard.putNumber("1 FL",modules[0].getDegrees());
    SmartDashboard.putNumber("2 FR",modules[1].getDegrees());
    SmartDashboard.putNumber("3 BL",modules[2].getDegrees());
    SmartDashboard.putNumber("4 BR",modules[3].getDegrees());*/

    modules[0].debug();
    modules[1].debug();
    modules[2].debug();
    modules[3].debug();

    // moduleStates[0].speedMetersPerSecond = Math.abs(modules[0].getDriveEncoderVelocity());
    // moduleStates[1].speedMetersPerSecond = Math.abs(modules[1].getDriveEncoderVelocity());
    // moduleStates[2].speedMetersPerSecond = Math.abs(modules[2].getDriveEncoderVelocity());
    // moduleStates[3].speedMetersPerSecond = Math.abs(modules[3].getDriveEncoderVelocity());

    odometry.update(
      getGyro(),
      swerveModulePositions
      );
      

    field2D.setRobotPose(getPose());

    if(isCalibrating){
      modules[0].calibrate("Front Left", offsetCalibration, driveCalibration, rotCalibration);
      modules[1].calibrate("Front Right", offsetCalibration, driveCalibration, rotCalibration);
      modules[2].calibrate("Back Left", offsetCalibration, driveCalibration, rotCalibration);
      modules[3].calibrate("Back Right", offsetCalibration, driveCalibration, rotCalibration);
    }

  }


  public void addTrajectoryToField2d(Trajectory traj) {
    field2D.getObject("traj").setTrajectory(traj);
  }

}