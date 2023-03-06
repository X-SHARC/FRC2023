package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;


public class ElevatorHome extends SequentialCommandGroup {
  /** Creates a new ElevatorHome. */
  Elevator elevator;

  //Carriage Homing will be added after carriage pid
  public ElevatorHome(Elevator elevator) {
    this.elevator = elevator;
   
    addCommands(
      new ElevatorCommand(elevator, 5).withTimeout(0.8),
      new RunCommand(()->elevator.elevatorHome(0.20), elevator).until(elevator::getHome),
      new RunCommand(()-> elevator.stop(), elevator),
      new RunCommand(()-> this.end(true))
    );
  }
}
