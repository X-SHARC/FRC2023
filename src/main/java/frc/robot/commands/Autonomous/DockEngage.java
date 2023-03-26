// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DockEngage extends CommandBase {
  /** Creates a new DockEngage. */
  Swerve dt = new Swerve(false);
  double currentPitch;
  double swerveFeed = 0;
  
  public DockEngage(Swerve swerve) {
    this.dt = swerve;

    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPitch = Math.abs(dt.getPitch());

    if(currentPitch>15) swerveFeed = 0.2;
    else if(currentPitch<15 &&currentPitch>7) swerveFeed = 0.1;
    else if(currentPitch<7 && currentPitch>3) swerveFeed = 0.05;
    else if(currentPitch <=3) swerveFeed = 0.01;
    else swerveFeed = 0;

    dt.drive(swerveFeed * Constants.Swerve.kMaxSpeed, 0, 0.0, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentPitch<=3 ? true: false;
  }
}
