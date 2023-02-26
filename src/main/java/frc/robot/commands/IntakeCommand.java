// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Intake;
import frc.robot.RobotState;
import frc.robot.RobotState.GamePiece;
public class IntakeCommand extends CommandBase {
  Intake intake;
  Joystick operator;
  static RobotState currentRobotState = RobotState.getInstance();

  public IntakeCommand(Intake intake, Joystick operator) {
    this.intake = intake;
    this.operator = operator;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    //INTAKE IDLING WILL BE ARRANGED
    if (RobotState.currentGamePiece == GamePiece.CONE){
      new JoystickButton(operator, 5).whileTrue(new RunCommand(()->intake.ejectCone())).whileFalse(new RunCommand(()->intake.stop()));
      new JoystickButton(operator, 6).whileTrue(new RunCommand(()->intake.grabCone())).whileFalse(new RunCommand(()->intake.stop()));
    }

    else if (RobotState.currentGamePiece == GamePiece.CUBE){
      new JoystickButton(operator, 5).whileTrue(new RunCommand(()->intake.ejectCube())).whileFalse(new RunCommand(()->intake.stop()));
      new JoystickButton(operator, 6).whileTrue(new RunCommand(()->intake.grabCube())).whileFalse(new RunCommand(()->intake.stop()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
