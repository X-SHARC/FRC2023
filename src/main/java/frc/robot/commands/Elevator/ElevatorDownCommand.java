// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;

public class ElevatorDownCommand extends CommandBase {
  Elevator elevator;
  
  public ElevatorDownCommand(Elevator elevator) {
    this.elevator = elevator;
    //addRequirements(elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      if (elevator.bottomLimitSwitch.get() == true){
          elevator.elevatorDown();}
      else{
          elevator.stop();
        }
      }
  

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
