// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Swerve;

public class SwerveDriveCommand extends CommandBase {
  XboxController joystick;
  Swerve swerveSubsystem;
  Joystick driver;

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(10);

  double scale = 1;

    public SwerveDriveCommand(Swerve sw, XboxController joystick) {
      this.swerveSubsystem = sw;
      this.joystick = joystick;
      addRequirements(swerveSubsystem);
    }


    public SwerveDriveCommand(Swerve sw, Joystick driver){
      this.swerveSubsystem = sw;
      this.driver = driver;

      addRequirements(swerveSubsystem);
    }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(RobotState.isElevated() || joystick.getRawButton(6)){
      scale = 0.38;
    }
    else scale = 1;

    if(scale == 0.45){
      joystick.setRumble(RumbleType.kRightRumble, 0.45);
      joystick.setRumble(RumbleType.kLeftRumble, 0.45);
    } 
      
    else{
      joystick.setRumble(RumbleType.kRightRumble, 0);
      joystick.setRumble(RumbleType.kLeftRumble, 0);
    } 
      

    double xSpeed = xSpeedLimiter.calculate(
      (Math.abs(joystick.getLeftY()) < 0.1) ? 0 : joystick.getLeftY())
      * Constants.Swerve.kMaxSpeed * scale;

    
    double ySpeed = ySpeedLimiter.calculate(
      (Math.abs(joystick.getLeftX()) <  0.1) ? 0 : joystick.getLeftX())
      * Constants.Swerve.kMaxSpeed * scale;
     
    double rot = -rotLimiter.calculate(
      (Math.abs(joystick.getRawAxis(4)) < 0.1) ? 0 : joystick.getRawAxis(4))
      * Constants.Swerve.kMaxAngularSpeed * scale;
    
    if(!RobotState.isBlueAlliance()){
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    swerveSubsystem.drive(xSpeed, ySpeed, rot, true, true);
  }

}
