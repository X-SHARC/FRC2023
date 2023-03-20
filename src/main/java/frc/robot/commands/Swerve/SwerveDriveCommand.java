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

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  // TODO not using them currently, try out and see if you want to keep them for comp
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2.5);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2.5);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4.5);

  double scale = 1;
  double scale2= 0.4;
  double slowScale = 0.2;

  
    /** Creates a new SwerveDriveCommand. */
    public SwerveDriveCommand(Swerve sw, XboxController joystick) {
      this.swerveSubsystem = sw;
      this.joystick = joystick;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(sw);
    }


    public SwerveDriveCommand(Swerve sw, Joystick driver){
      this.swerveSubsystem = sw;
      this.driver = driver;

      addRequirements(swerveSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    scale = RobotState.isElevated() ? 0.45 : 1;

    if(scale == 0.45){
      joystick.setRumble(RumbleType.kRightRumble, 0.45);
      joystick.setRumble(RumbleType.kLeftRumble, 0.45);
    } 
      
    else{
      joystick.setRumble(RumbleType.kRightRumble, 0);
      joystick.setRumble(RumbleType.kLeftRumble, 0);
    } 
      

    double xSpeed = xSpeedLimiter.calculate(
      MathUtil.applyDeadband(joystick.getLeftY(), 0.15) * Constants.Swerve.kMaxSpeed 
      )
      * scale;

    
    double ySpeed = ySpeedLimiter.calculate(
      MathUtil.applyDeadband(joystick.getLeftX(), 0.15) * Constants.Swerve.kMaxSpeed
      )
      * scale;
     
    double rot = -rotLimiter.calculate(
      MathUtil.applyDeadband(joystick.getRightX(), 0.15) * Constants.Swerve.kMaxAngularSpeed
      )
      * scale;
    
    if(!RobotState.isBlueAlliance()){
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    swerveSubsystem.drive(xSpeed, ySpeed, rot, true, true);
  }

}
