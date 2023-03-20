// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase {
  Elevator elevator;
  double distance;
  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(Elevator elevator, double distance) {
    this.elevator = elevator;
    this.distance = distance;
    //! ADD REQUIREMENTS DO NOT WORK FIX ME
    addRequirements(elevator);
  }
  

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.setDistance(distance);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return elevator.isAtSetpoint();
  }
}
