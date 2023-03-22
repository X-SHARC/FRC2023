// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  Elevator elevator = new Elevator();
  Carriage carriage = new Carriage();
  Intake intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final static Joystick operator = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    JoystickButton elevatorUp = new JoystickButton(operator, 1);
    elevatorUp.whileTrue(new RunCommand(()-> elevator.elevatorUp(), elevator));
    elevatorUp.whileFalse(new RunCommand(()-> elevator.stop(), elevator));

    JoystickButton elevatorDown = new JoystickButton(operator, 2);
    elevatorDown.whileTrue
    (new RunCommand(()-> elevator.elevatorDown(), elevator));
    elevatorUp.whileFalse(new RunCommand(()-> elevator.stop(), elevator));

    JoystickButton elevatorpid = new JoystickButton(operator, 5);
    elevatorpid.whileTrue(new RunCommand(()-> elevator.setDistance(100), elevator));
    elevatorpid.whileFalse(new RunCommand(()-> elevator.stop(), elevator));

    JoystickButton elevatorpid2 = new JoystickButton(operator, 6);
    elevatorpid2.whileTrue(new RunCommand(()-> elevator.setDistance(10), elevator));
    elevatorpid2.whileFalse(new RunCommand(()->
    elevator.stop(), elevator));

    JoystickButton intakeUp = new JoystickButton(operator, 3);
    intakeUp.whileTrue(new RunCommand(()-> carriage.intakeUp(), carriage));
    intakeUp.whileFalse(new RunCommand(()-> carriage.stop(), carriage));

    JoystickButton intakeDown = new JoystickButton(operator, 4);
    intakeDown.whileTrue(new RunCommand(()-> carriage.intakeDown(), carriage));
    intakeDown.whileFalse(new RunCommand(()-> carriage.stop(), carriage));

   
    JoystickButton all = new JoystickButton(operator, 7);
    all.whileTrue(
      new SequentialCommandGroup(
        new RunCommand(()-> elevator.setDistance(100), elevator).withTimeout(1)
        .alongWith(new RunCommand(()-> carriage.setDegrees(48), carriage).withTimeout(0.5)),
        new InstantCommand(()-> carriage.stop(), carriage),
        new RunCommand(()-> intake.ejectCube(), intake).withTimeout(0.3),
        new InstantCommand(()-> intake.stop(), intake),
        new RunCommand(()-> elevator.setDistance(20), elevator).withTimeout(1),
        new RunCommand(()-> elevator.setDistance(100), elevator).withTimeout(1),
        new RunCommand(()-> elevator.setDistance(20), elevator).withTimeout(1),
        new InstantCommand(()-> elevator.stop(), elevator)
        )
    );

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}

