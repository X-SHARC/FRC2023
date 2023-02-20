// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.Elevator;

public class TakeFromGroundCommand extends CommandBase {

  Intake intake;
  Slider slider;
  Elevator elevator;
  double distance;

  /** Creates a new TakeFromGroundCommand. */
  public TakeFromGroundCommand(Intake intake, Slider slider, Elevator elevator) {
    this.intake = intake;
    this.slider = slider;
    this.elevator = elevator;
    addRequirements(intake, slider, elevator);
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = 12;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetEncoder();
    this.distance = 12;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setDistance(12);
    intake.grabObject();
    slider.sliderForward();
    elevator.elevatorDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    slider.stop();
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
