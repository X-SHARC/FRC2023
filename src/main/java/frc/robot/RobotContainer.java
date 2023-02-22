// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.CarriageCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driver = new XboxController(0);
  private final Joystick operator = new Joystick(1);
 // private final Joystick driverJoystick = new Joystick(0);

  //Subsystems
  Swerve swerveDrivetrain = new Swerve(true);
  Elevator elevator = new Elevator();
  Intake intake = new Intake();
  Carriage carriage = new Carriage();

  //Commands 
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  ElevatorCommand elevatorCommand = new ElevatorCommand(elevator, 70);
  ElevatorUpCommand elevatorUpCommand = new ElevatorUpCommand(elevator);
  ElevatorDownCommand elevatorDownCommand = new ElevatorDownCommand(elevator);
  CarriageCommand carriageCommand = new CarriageCommand(carriage, 15);

      public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    boolean a = RobotState.getTripping();
    //swerveDrivetrain.setDefaultCommand(driveCommand);
    JoystickButton[] elevatorButtons = {
      new JoystickButton(driver, 1),
      new JoystickButton(driver, 2),
      new JoystickButton(driver, 3),
      new JoystickButton(driver, 4),
    };

    JoystickButton elevator1 = new JoystickButton(operator, 1);
    elevator1.whileTrue(elevatorUpCommand);
  //  elevator1.whileTrue(new RunCommand(()-> elevator.elevatorUp(), elevator));
   // elevator1.whileFalse(new RunCommand(()-> elevator.stop(), elevator));
  

    JoystickButton elevator2 = new JoystickButton(operator,2);
    elevator2.whileTrue(elevatorDownCommand);
//   elevator2.whileTrue(new RunCommand(()-> elevator.elevatorDown(), elevator));
  // elevator2.whileFalse(new RunCommand(()-> elevator.stop(), elevator));

     JoystickButton elevator3 = new JoystickButton(operator,5);
   elevator3.whileTrue(new ElevatorCommand(elevator, 70));
/* 
   JoystickButton elevator4 = new JoystickButton(driver,4);
   elevator4.whileTrue(new RunCommand(()-> elevator.setDistance(15), elevator));
   elevator4.whileFalse(new RunCommand(()-> elevator.stop(), elevator));
*/
   JoystickButton carriage1 = new JoystickButton(operator, 7);
   carriage1.whileTrue(new RunCommand(()-> carriage.intakeUp(), carriage));
   carriage1.whileFalse(new RunCommand(()-> carriage.stop(), carriage));

   JoystickButton carriage2 = new JoystickButton(operator,8);
   carriage2.whileTrue(new RunCommand(()-> carriage.intakeDown(), carriage));
   carriage2.whileFalse(new RunCommand(()-> carriage.stop(), carriage));

   JoystickButton carriage3 = new JoystickButton(operator, 11);
   carriage3.onTrue(carriageCommand);

   JoystickButton intake1 = new JoystickButton(operator, 3);
   intake1.whileTrue(new RunCommand(()-> intake.grabCone(), intake));
   intake1.whileFalse(new RunCommand(()-> intake.stop(), intake));

   JoystickButton intake2 = new JoystickButton(operator,4);
   intake2.whileTrue(new RunCommand(()-> intake.grabCube(), intake));
   intake2.whileFalse(new RunCommand(()-> intake.stop(), intake));
   swerveDrivetrain.setDefaultCommand(driveCommand);

   JoystickButton encoderReset = new JoystickButton(operator, 6);
   encoderReset.onTrue(new RunCommand(() -> elevator.resetEncoder(), elevator));
  }
  

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}