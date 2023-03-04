// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.management.RuntimeMXBean;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.RobotState.GamePiece;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ElevatorHome;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTo2Command extends SequentialCommandGroup {
  Elevator elevator;
  Intake intake;
  Carriage carriage;
  double distance;
  RunCommand intakeCommand = RobotState.getGamePiece() == GamePiece.CONE ? new RunCommand(()->intake.ejectCone())  : new RunCommand(()->intake.ejectCube());
  
  /** Creates a new ScoreTo2Command. */
  public ScoreTo2Command(Elevator elevator, Intake intake, Carriage carriage) {
    this.elevator = elevator;
    this.intake = intake;
    this.carriage = carriage;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new CarriageCommand(carriage, -35).withTimeout(1.3),
      new ElevatorCommand(elevator, 75).withTimeout(1),
      new CarriageCommand(carriage, -75).withTimeout(1),
      intakeCommand.withTimeout(2.0),
      new RunCommand(()->intake.stop()).withTimeout(0.5),
      new CarriageCommand(carriage, -30).withTimeout(1.3),
      new ElevatorHome(elevator).withTimeout(2)
      );
  }
}
