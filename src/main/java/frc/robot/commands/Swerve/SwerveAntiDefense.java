// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SwerveAntiDefense extends CommandBase {
  /** Creates a new SwerveAntiDefense. */
  Swerve swerve;
  private SwerveModuleState desiredStates[] = {
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(-45)),
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(-45)),
  };

  public SwerveAntiDefense(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setModuleStates(desiredStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
