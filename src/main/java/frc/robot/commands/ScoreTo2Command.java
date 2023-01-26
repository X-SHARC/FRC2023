// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Slider;

public class ScoreTo2Command extends CommandBase {
  Elevator elevator;
  Intake intake;
  Slider slider;
  double distance;

  /** Creates a new ScoreCommand. */
  public ScoreTo2Command(Elevator elevator, Intake intake, Slider slider) {
    this.elevator = elevator;
    this.slider = slider;
    this.intake = intake;
    addRequirements(intake, slider, elevator);
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = 87;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetEncoder();
    this.distance = 87;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setDistance(87);
    elevator.elevatorUp();
    slider.sliderForward();
    intake.dropObject();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    elevator.stop();
    slider.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
