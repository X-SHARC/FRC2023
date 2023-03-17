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
  private boolean isCalibrating;
  private boolean offsetCalibration = true;
  private boolean driveCalibration = false;

  private Rotation2d fieldAngle = new Rotation2d();

  private final Field2d field2D = new Field2d();
  
  /**
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

  public Pigeon2 pigeon = new Pigeon2(23);

  final boolean invertAllModules = true;
  private double kP = 0.00156;
  private SwerveModule[] modules = new SwerveModule[] {

    new SwerveModule(
      "FL", 
      new TalonFX(17),
      new TalonFX(13),
      new CANCoder(4),
      false,
      true,
      -298), //! Front Left

    new SwerveModule(
      "FR",
      new TalonFX(14),
      new TalonFX(18),
      new CANCoder(3),
      false,
      true,
      -40), //! Front Right

    new SwerveModule(
      "RL",
      new TalonFX(11),
      new TalonFX(16),
      new CANCoder(2), 
      false,
      true,
      -35), //! Back Left

    new SwerveModule(
      "RR",
      new TalonFX(10),
      new TalonFX(12),
      new CANCoder(1),
      false,
      true,
      -11)  //! Back Right
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
    SmartDashboard.putData("Field", field2D);
  }
  
  public Rotation2d getGyro(){
    return Rotation2d.fromDegrees(
        getGyroDouble()
        );
  }

  public double getGyroDouble(){
    return Math.IEEEremainder(pigeon.getYaw(), 360) * (Constants.kGyroReversed ? 1.0 : -1.0); 
  }

  public void resetPigeon(){
    pigeon.setYaw(0);
  }

  public double getPitch(){
    return pigeon.getPitch();
  }

  public SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    Constants.Swerve.kinematics,
    getGyro(),
    swerveModulePositions
    );

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
      module.resetDriveEncoder();
    }
  }

  public void resetRotEncoders(){
    for (int i = modules.length-1; i >= 0; i--) {
      SwerveModule module = modules[i];
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
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean openLoop) {
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
      if(openLoop) module.setDesiredState(state);
      else module.setClosedLoop(state);
    }
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
    swerveModulePositions = new SwerveModulePosition[] {
      modules[0].getModulePosition(),
      modules[1].getModulePosition(),
      modules[2].getModulePosition(),
      modules[3].getModulePosition()
    };

    odometry.update(
      getGyro(),
      swerveModulePositions
      );

    field2D.setRobotPose(getPose());

    SmartDashboard.putNumber("Swerve Gyro Angle", getGyroDouble());
    SmartDashboard.putNumber("Swerve Field Offset", fieldAngle.getDegrees());
    
    SmartDashboard.putNumber("Odometry Pose X", getPose().getX());
    SmartDashboard.putNumber("Odometry Pose Y", getPose().getY());
    SmartDashboard.putNumber("Odometry Rot", getPose().getRotation().getRadians());

    if(isCalibrating){
      modules[0].calibrate("Front Left", offsetCalibration, driveCalibration);
      modules[1].calibrate("Front Right", offsetCalibration, driveCalibration);
      modules[2].calibrate("Back Left", offsetCalibration, driveCalibration);
      modules[3].calibrate("Back Right", offsetCalibration, driveCalibration);
    }

  }


  public void addTrajectoryToField2d(Trajectory traj) {
    field2D.getObject("traj").setTrajectory(traj);
  }

}