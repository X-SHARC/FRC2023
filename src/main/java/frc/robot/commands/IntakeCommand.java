// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.RobotState;
public class IntakeCommand extends CommandBase {
  Intake intake;

  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
   if(RobotState.getIntaking() == RobotState.IntakeState.IDLE){

      if(RobotState.getGamePiece() == RobotState.GamePiece.CONE){
        intake.coneidle();
      }
      else intake.stop();
    }
    
    //INTAKE IDLING WILL BE ARRANGED
    else{
      if(RobotState.getIntaking() == RobotState.IntakeState.INTAKING){
        if(RobotState.getGamePiece() == RobotState.GamePiece.CONE){
          intake.grabCone();
        }
        else if(RobotState.getGamePiece() == RobotState.GamePiece.CUBE){
          intake.grabCube();
        }
        else intake.stop();
      }
      else if(RobotState.getIntaking() == RobotState.IntakeState.EJECTING){
        if(RobotState.getGamePiece() == RobotState.GamePiece.CONE){
          intake.ejectCone();
        }
        else if(RobotState.getGamePiece() == RobotState.GamePiece.CUBE){
          intake.ejectCube();
        }
        else intake.stop();
      }
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
