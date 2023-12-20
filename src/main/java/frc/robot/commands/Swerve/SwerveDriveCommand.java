// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDriveCommand extends CommandBase {
  XboxController joystick;
  Swerve swerveSubsystem;

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(10);


  double scale = 0.5;

    public SwerveDriveCommand(Swerve sw, XboxController driver){
      this.swerveSubsystem = sw;
      this.joystick = driver;

      addRequirements(swerveSubsystem);
    }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double xSpeed = xSpeedLimiter.calculate(
      (Math.abs(joystick.getLeftY()) < 0.1) ? 0 : joystick.getLeftY())
      * Constants.Swerve.kMaxSpeed * scale;

    double ySpeed = ySpeedLimiter.calculate(
      (Math.abs(joystick.getLeftX()) <  0.1) ? 0 : joystick.getLeftX())
      * Constants.Swerve.kMaxSpeed * scale;
     
    double rot = rotLimiter.calculate(
      (Math.abs(joystick.getRightX()) < 0.1) ? 0 : joystick.getRightX())
      * Constants.Swerve.kMaxAngularSpeed * scale;

    swerveSubsystem.drive(xSpeed, ySpeed, rot, false, true);
  }

}
