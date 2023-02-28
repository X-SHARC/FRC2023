// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevator;

public class ElevatorPOV extends CommandBase {
  /** Creates a new ElevatorPOV. */
  Joystick operator;
  Elevator elevator;
  ElevatorUpCommand up;
  ElevatorDownCommand down;
  ElevatorHome home;
  ElevatorCommand pid;
  public ElevatorPOV(Joystick operator, Elevator elevator, ElevatorUpCommand up, ElevatorDownCommand down, ElevatorHome home, ElevatorCommand pid) {
    this.elevator = elevator;
    this.operator = operator;
    this.up = up;
    this.down = down;
    this.home = home;
    this.pid = pid;

    //addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(operator.getPOV()){
      case 0:
        CommandScheduler.getInstance().schedule(pid);
        break;
      case 180:
        CommandScheduler.getInstance().schedule(home);        
        break;
    
      default:
        up.end(true);
        down.end(true);
        home.end(true);
        pid.end(true);
        elevator.stop();

    if (Math.abs(operator.getY())>0.15){
      elevator.percent(operator.getY()*0.5);
    }
    }
    //new JoystickButton(operator, 3).onTrue(home);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
