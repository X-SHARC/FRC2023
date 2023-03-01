// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.CarriageCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ElevatorDownCommand;
import frc.robot.commands.Elevator.ElevatorHome;
import frc.robot.commands.Elevator.ElevatorUpCommand;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.lib.drivers.WS2812Driver;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static XboxController driver = new XboxController(0);
  private final static Joystick operator = new Joystick(1);

  //Subsystems
  static Swerve swerveDrivetrain = new Swerve(true);
  static Elevator elevator = new Elevator(operator);
  static Intake intake = new Intake();
  static Carriage carriage = new Carriage();

  //TODO: Change the LED length
  static WS2812Driver rgbLED = new WS2812Driver(0, 13);

  //Commands 
  static SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  static ElevatorCommand elevatorCommand = new ElevatorCommand(elevator, 110);
  static ElevatorUpCommand elevatorUpCommand = new ElevatorUpCommand(elevator);
  static ElevatorDownCommand elevatorDownCommand = new ElevatorDownCommand(elevator);
  static ElevatorHome elevatorHome = new ElevatorHome(elevator);
  static CarriageCommand carriageCommand = new CarriageCommand(carriage, 45);
  static IntakeCommand intakeCommand = new IntakeCommand(intake,operator);
  //static ElevatorPOV elevatorPOV = new ElevatorPOV(operator, elevator, elevatorUpCommand, elevatorDownCommand, elevatorHome, elevatorCommand);

  public final static PowerDistribution pdh = new PowerDistribution();

  public RobotContainer() {
    // Configure the trigger bindings
    /*RobotState.setGamePiece(GamePiece.EMPTY);
    RobotState.setIntakeState(IntakeState.STOP);
    RobotState.setSwerve(SwerveState.REST);
    RobotState.setElevatorLevel(ElevatorLevel.ZERO);*/
    configureBindings();
  }

  private void configureBindings() {
    boolean a = RobotState.getTripping();
    swerveDrivetrain.setDefaultCommand(driveCommand);
    JoystickButton elevator1 = new JoystickButton(operator, 8);
    elevator1.whileTrue(elevatorUpCommand);

    JoystickButton elevator2 = new JoystickButton(operator,7);
    elevator2.whileTrue(elevatorDownCommand);


    //TODO: add carriage default command: pid, home, limitations
   JoystickButton carriage1 = new JoystickButton(operator, 10);
   carriage1.whileTrue(new RunCommand(()-> carriage.intakeUp(), carriage));
   carriage1.whileFalse(new RunCommand(()-> carriage.stop(), carriage));

   JoystickButton carriage2 = new JoystickButton(operator,9);
   carriage2.whileTrue(new RunCommand(()-> carriage.intakeDown(), carriage));
   carriage2.whileFalse(new RunCommand(()-> carriage.stop(), carriage));

    /* 
   JoystickButton carriageButton = new JoystickButton(operator, 1);
   carriageButton.whileTrue(carriageCommand);

   JoystickButton resetCarriageButton = new JoystickButton(operator, 2);
   resetCarriageButton.onTrue(new RunCommand(()-> carriage.resetCarriageEncoder()));
  */

   //TODO: Change this button
   /*JoystickButton encoderReset = new JoystickButton(operator, 8);
   encoderReset.onTrue(new RunCommand(() -> elevator.resetEncoder(), elevator));*/


   //GAME PIECE SELECTOR BUTTONS
   /*JoystickButton cubeButton = new JoystickButton(operator, 3);
   JoystickButton coneButton = new JoystickButton(operator, 4);
   coneButton.whileTrue(new RunCommand(()->RobotState.setCone()));
   cubeButton.whileTrue(new RunCommand(()->RobotState.setCube()));*/

   JoystickButton intakeButton = new JoystickButton(operator, 6);
   intakeButton.whileTrue(new RunCommand(()->RobotState.setIntaking()));
   intakeButton.whileFalse(new RunCommand(()-> RobotState.setIntakeIdle()));

   JoystickButton ejectButton = new JoystickButton(operator, 5);
   ejectButton.whileTrue(new RunCommand(()->RobotState.setEjecting()));
   ejectButton.whileFalse(new RunCommand(()-> RobotState.setIntakeIdle()));
  

   intake.setDefaultCommand(intakeCommand);
   //elevator.setDefaultCommand(elevatorPOV);  
  }
  

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}