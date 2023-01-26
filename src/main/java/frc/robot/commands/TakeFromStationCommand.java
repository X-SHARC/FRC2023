// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Slider;

public class TakeFromStationCommand extends CommandBase {
  Intake intake;
  Slider slider;
  Elevator elevator;
  double distance;

  /** Creates a new TakeFromStationCommand. */
  public TakeFromStationCommand(Intake intake, Slider slider, Elevator elevator) {
    this.intake = intake;
    this.slider = slider;
    this.elevator = elevator;
    addRequirements(intake, slider, elevator);
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = 95;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetEncoder();
    this.distance = 97;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setDistance(97);
    intake.grabObject();
    slider.sliderForward();
    elevator.elevatorUp();
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
