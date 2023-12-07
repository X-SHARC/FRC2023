// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
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

  double scale = 0.6;
  double scale2= 0.5;
  
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
    swerveSubsystem.resetRotEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    scale = Math.abs(joystick.getRightTriggerAxis()) < 0.4 ? 1: scale2;

    final var xSpeed = xSpeedLimiter.calculate(
      (Math.abs(joystick.getLeftY()) < 0.1) ? 0 : joystick.getLeftY())
      * Constants.Swerve.kMaxSpeed * scale;

    
    final var ySpeed = ySpeedLimiter.calculate(
      (Math.abs(joystick.getLeftX()) <  0.1) ? 0 : joystick.getLeftX())
      * Constants.Swerve.kMaxSpeed * scale;
     
    final var rot = rotLimiter.calculate(
      (Math.abs(joystick.getRightX()) < 0.1) ? 0 : joystick.getRightX())
      * Constants.Swerve.kMaxAngularSpeed * scale;


    //double[] speeds ={xSpeed, ySpeed, rot}; 
    //SmartDashboard.putNumberArray("controller speeds", speeds);
    swerveSubsystem.drive(xSpeed, ySpeed, rot, true);
  }

  /*public InstantCommand swerveDrive(){
    //Axis: Left Y: 1, Left X: 0, Right Y: 4, Right X: 2 ????
    scale = Math.abs(driver.getRawAxis(1)) < 0.4 ? 1: scale2;

    final var xSpeed = xSpeedLimiter.calculate(
      (Math.abs(driver.getRawAxis(1)) < 0.1) ? 0 : driver.getRawAxis(1))
      * Constants.Swerve.kMaxSpeed * scale;

    
    final var ySpeed = ySpeedLimiter.calculate(
      (Math.abs(driver.getRawAxis(0)) <  0.1) ? 0 : driver.getRawAxis(0))
      * Constants.Swerve.kMaxSpeed * scale;
     
    final var rot = rotLimiter.calculate(
      (Math.abs(driver.getRawAxis(2)) < 0.1) ? 0 : driver.getRawAxis(2))
      * Constants.Swerve.kMaxAngularSpeed * scale;

    double[] speeds ={xSpeed, ySpeed, rot}; 
    SmartDashboard.putNumberArray("controller speeds", speeds);
    swerveSubsystem.drive(xSpeed, ySpeed, rot, true);
    return new InstantCommand(()->swerveDrive());
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }*/
}
