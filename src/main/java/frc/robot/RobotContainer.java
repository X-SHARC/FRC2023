package frc.robot;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.RobotState.GamePiece;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Autonomous.AutoAlign;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ElevatorDownCommand;
import frc.robot.commands.Elevator.ElevatorHome;
import frc.robot.commands.Elevator.ElevatorUpCommand;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.lib.drivers.WS2812Driver;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static XboxController driver = new XboxController(0);
  private final static Joystick operator = new Joystick(1);

  //Subsystems
  Swerve swerveDrivetrain = new Swerve(true);
  Elevator elevator = new Elevator();
  Intake intake = new Intake();
  Carriage carriage = new Carriage();
  WS2812Driver leftLED = new WS2812Driver(0, 44);
  

  //Commands 
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  ElevatorCommand elevatorCommand = new ElevatorCommand(elevator, 75);
  ElevatorUpCommand elevatorUpCommand = new ElevatorUpCommand(elevator);
  ElevatorDownCommand elevatorDownCommand = new ElevatorDownCommand(elevator);
  ElevatorHome elevatorHome = new ElevatorHome(elevator);
  IntakeCommand intakeCommand = new IntakeCommand(intake,operator);
  SharcTrajectory trajectoryGenerator = new SharcTrajectory();

  PowerDistribution pdh = new PowerDistribution();
  AutoAlign autoAlign = new AutoAlign(swerveDrivetrain);

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    swerveDrivetrain.setDefaultCommand(driveCommand);
    JoystickButton elevator1 = new JoystickButton(operator, 8);
    elevator1.whileTrue(elevatorUpCommand);

    JoystickButton elevator2 = new JoystickButton(operator,7);
    elevator2.whileTrue(elevatorDownCommand);

    JoystickButton elevatorhome = new JoystickButton(driver,7);
    elevatorhome.whileTrue(elevatorHome); 

    JoystickButton carriagepid = new JoystickButton(operator, 5);
   carriagepid.onTrue(new InstantCommand(()-> carriage.setSetpoint(30))); 
  
   // denenecek + button atama kontrol
   JoystickButton carriagecommand = new JoystickButton(operator, 3);
   carriagecommand.onTrue(new InstantCommand(()-> carriage.setSetpoint(5)));

   JoystickButton secondLevel = new JoystickButton(driver, 2);
   secondLevel.onTrue(
    new SequentialCommandGroup(
      //new RunCommand(()->carriage.setSetpoint(15), carriage).withTimeout(0.5),
      new ElevatorCommand(elevator, RobotState.getGamePiece() == GamePiece.CONE ? 60: 75)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(RobotState.getGamePiece() == GamePiece.CUBE ? 68:38))),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
      new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
      new ElevatorHome(elevator).withTimeout(1.8)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(5)))
    ).beforeStarting(()->carriage.setSetpoint(15))
   );

   JoystickButton thirdLevel = new JoystickButton(driver, 4);
   thirdLevel.onTrue(
    new SequentialCommandGroup(
      //new RunCommand(()->carriage.setSetpoint(15), carriage).withTimeout(0.5),
      new ElevatorCommand(elevator, RobotState.getGamePiece() == GamePiece.CONE ? 60: 75)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(RobotState.getGamePiece() == GamePiece.CUBE ? 68:38))),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
      new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
      new ElevatorHome(elevator).withTimeout(1.8)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(5)))
    )
    .beforeStarting(
      Commands.parallel(
       new InstantCommand( ()->carriage.setSetpoint(15)),
       new WaitCommand(0.2)
       )
      )
   );

/* DRIVER AUTOSCORE BUTTONS -- DEPRECATED
   JoystickButton secondLevelcone = new JoystickButton(driver, 1);
   secondLevelcone.onTrue(
    new SequentialCommandGroup(
      new InstantCommand(()->carriage.setSetpoint(15)),
      new ElevatorCommand(elevator, 60).withTimeout(1)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(38))),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
      new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
      new ElevatorHome(elevator).withTimeout(1.8)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(5)))
    ).beforeStarting(()->carriage.setSetpoint(15))
   );
   secondLevelcone.onFalse(new RunCommand(()->elevator.stop()));

   JoystickButton secondLevelcube = new JoystickButton(driver, 2);
   secondLevelcube.onTrue(
    new SequentialCommandGroup(
      new InstantCommand(()->carriage.setSetpoint(38)),
      new ElevatorCommand(elevator, 75).withTimeout(1)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(68))),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
      new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
      new ElevatorHome(elevator).withTimeout(1.8)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(5)))
    ).beforeStarting(()->carriage.setSetpoint(15))
   );

   secondLevelcone.onFalse(new RunCommand(()->elevator.stop()));

   JoystickButton thirdLevelcube = new JoystickButton(driver, 3);
   thirdLevelcube.onTrue(
    new SequentialCommandGroup(
      new InstantCommand(()->carriage.setSetpoint(15)),
      new ElevatorCommand(elevator, 101).withTimeout(1)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(32))),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
      new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
      new ElevatorHome(elevator).withTimeout(1.8)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(5)))
    ).beforeStarting(()->carriage.setSetpoint(15))
   );

   thirdLevelcube.onFalse(new RunCommand(()->elevator.stop()));
 
   JoystickButton thirdLevelcone = new JoystickButton(driver, 4);
   thirdLevelcone.onTrue(
    new SequentialCommandGroup(
      new InstantCommand(()->carriage.setSetpoint(48)),
      new ElevatorCommand(elevator, 112).withTimeout(1)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(36))),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.3),
      new InstantCommand(()->RobotState.setIntakeIdle()),
      new ElevatorHome(elevator).withTimeout(1.8)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(5)))
      ).beforeStarting(()->carriage.setSetpoint(15))
   );
   thirdLevelcone.onFalse(new RunCommand(()->elevator.stop())); */
   /*JoystickButton takeFromStation = new JoystickButton(operator, 10);
   takeFromStation.onTrue(
    new SequentialCommandGroup(
      new InstantCommand(()->carriage.setSetpoint(15)),
      new ElevatorCommand(elevator, 110)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(60)))
      .raceWith(new InstantCommand(()->RobotState.setIntaking())).withTimeout(1)
      .andThen(new InstantCommand(()->carriage.setSetpoint(15)))
      .andThen(new ElevatorHome(elevator))
      )
   );
   takeFromStation.onFalse(new InstantCommand(()->RobotState.setIntakeIdle()));
*/
   JoystickButton turn = new JoystickButton(driver, 5);
   turn.whileTrue(autoAlign);

   //TODO: Change this button
   JoystickButton encoderReset = new JoystickButton(operator, 12);
   encoderReset.onTrue(new RunCommand(() -> elevator.resetEncoder(), elevator));

   JoystickButton intakeButton = new JoystickButton(operator, 2);
   intakeButton.whileTrue(
    new RunCommand(()->RobotState.setIntaking())
    .beforeStarting(new InstantCommand(()->carriage.setSetpoint(RobotState.getGamePiece()==GamePiece.CONE? 78:100)))
    );

   intakeButton.whileFalse(
    new RunCommand(()-> RobotState.setIntakeIdle())
    .beforeStarting(new InstantCommand(()->carriage.setSetpoint(3)))
    );

   JoystickButton ejectButton = new JoystickButton(operator, 1);
   ejectButton.whileTrue(new RunCommand(()->RobotState.setEjecting()).beforeStarting(
    new InstantCommand(()->carriage.setSetpoint(RobotState.getGamePiece() == GamePiece.CONE ? 50:75 ))
   ));
   ejectButton.whileFalse(
    new RunCommand(()-> RobotState.setIntakeIdle())
    .beforeStarting(new RunCommand(()->carriage.setSetpoint(3)).withTimeout(0.2))
    );

   intake.setDefaultCommand(intakeCommand);
  }
  

  public Command getAutonomousCommand() {
    return trajectoryGenerator.getControllerCommand(swerveDrivetrain, "eglence");
  }
}