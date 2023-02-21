// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.RobotState;
import frc.robot.RobotState.GamePiece;
public class IntakeCommand extends CommandBase {
  Intake intake;
  static RobotState currentRobotState = RobotState.getInstance();

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState.setIntaking();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotState.getInstance().currentGamePiece == GamePiece.CONE){
      intake.grabCone();
    }
    else if (RobotState.getInstance().currentGamePiece == GamePiece.CUBE){
      intake.grabCube();
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
