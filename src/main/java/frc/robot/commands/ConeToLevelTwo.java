package frc.robot.commands;
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
public class ConeToLevelTwo extends SequentialCommandGroup {
  Elevator elevator;
  Intake intake;
  Carriage carriage;
  double distance;
  RunCommand intakeCommand;
  //Array Structure: [setpoint, timeout]

  
  /** Creates a new ScoreTo2Command. */
  public ConeToLevelTwo(Elevator elevator, Intake intake, Carriage carriage) {
    this.elevator = elevator;
    this.intake = intake;
    this.carriage = carriage;
    RunCommand intakeCommand = RobotState.getGamePiece() == GamePiece.CONE ? new RunCommand(()->intake.ejectCone())  : new RunCommand(()->intake.ejectCube());

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  

    addCommands(
      new CarriageCommand(carriage, -30).withTimeout(0.1),
      new ElevatorCommand(elevator, 60).withTimeout(1)
      .alongWith(new CarriageCommand(carriage, -38).withTimeout(0.4)),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.3),
      new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
      new ElevatorHome(elevator).withTimeout(1.8)
      .alongWith(new CarriageCommand(carriage, -15).withTimeout(1.3))

      );
  }
}
