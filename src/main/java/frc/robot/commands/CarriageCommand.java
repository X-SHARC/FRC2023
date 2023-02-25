// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// not complete 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carriage;

public class CarriageCommand extends CommandBase {
  Carriage carriage;
  double setpoint;
  /** Creates a new CarriageCommand. */
  public CarriageCommand(Carriage carriage, double setpoint) {
    this.carriage = carriage;
    this.setpoint = setpoint;
    addRequirements(carriage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    carriage.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    carriage.setDegrees(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    carriage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
