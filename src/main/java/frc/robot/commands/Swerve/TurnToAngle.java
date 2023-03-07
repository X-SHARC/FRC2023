// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAgnle. */
  Swerve swerve;
  //kp = 0.0075
  PIDController turnPID = new PIDController(0.0059, 0, 0);

  double angleSetpoint;
  
  public TurnToAngle(Swerve swerve, double angleSetpoint) {
    this.swerve = swerve;
    this.angleSetpoint = angleSetpoint;
    addRequirements(swerve);
    turnPID.setTolerance(5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // ! NOT FIELD RELATIVE
    swerve.drive(0, 0, 
    turnPID.calculate(swerve.pigeon.getYaw(), angleSetpoint) * Constants.Swerve.kMaxAngularSpeed
    , false);

    SmartDashboard.putBoolean("atSetpoint", turnPID.atSetpoint());
    SmartDashboard.putNumber("Setpoint", angleSetpoint);
    SmartDashboard.putNumber("PigeonYaw", swerve.pigeon.getYaw());

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
