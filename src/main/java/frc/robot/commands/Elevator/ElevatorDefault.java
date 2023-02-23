// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class ElevatorDefault extends CommandBase {
  /** Creates a new ElevatorDefault. */
  Elevator elevator;
  Joystick operator;
  Carriage carriage;
  Intake intake;
  public ElevatorDefault(Elevator elevator,Joystick operator, Carriage carriage,Intake intake) {
    this.elevator = elevator;
    this.operator = operator;
    this.carriage = carriage;
    this.intake = intake;
    
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(operator.getPOV()==0) new ElevatorUpCommand(elevator);
    if(operator.getPOV()==180) new ElevatorDownCommand(elevator);
    if(operator.getPOV()==90) new ElevatorHome(elevator);
    if(operator.getRawButton(1)) new RunCommand(()-> carriage.intakeDown(), carriage);
    else new RunCommand(()-> carriage.stop(), carriage);
    if(operator.getRawButton(4)) new RunCommand(()-> carriage.intakeUp(), carriage);
    new RunCommand(()-> carriage.stop(), carriage);
    if(operator.getRawButton(6)) new RunCommand(()-> intake.grabCone(), intake);
    new RunCommand(()-> intake.stop(), intake);
    if(operator.getRawButton(5)) new RunCommand(()-> intake.grabCube(), intake);
    new RunCommand(()-> intake.stop(), intake);
    if(operator.getPOV()==270) new ElevatorCommand(elevator, 100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    carriage.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
