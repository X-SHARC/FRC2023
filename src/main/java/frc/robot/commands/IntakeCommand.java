// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
      if (operator.getRawButton(5)) intake.ejectCone();
      else if (operator.getRawButton(6)) intake.grabCone();
      else intake.idle();
    }

    else if (RobotState.currentGamePiece == GamePiece.CUBE){
      if (operator.getRawButton(5)) intake.ejectCube();
      else if (operator.getRawButton(6)) intake.grabCube();
      else intake.idle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
