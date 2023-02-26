// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.RobotState.ElevatorLevel;
import frc.robot.RobotState.GamePiece;
import frc.robot.RobotState.IntakeState;
import frc.robot.RobotState.SwerveState;
import frc.robot.commands.CarriageCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ElevatorDownCommand;
import frc.robot.commands.Elevator.ElevatorHome;
import frc.robot.commands.Elevator.ElevatorPOV;
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
  public RobotState robotState = RobotState.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driver = new XboxController(0);
  private final Joystick operator = new Joystick(1);
 // private final Joystick driverJoystick = new Joystick(0);

  //Subsystems
  Swerve swerveDrivetrain = new Swerve(true);
  Elevator elevator = new Elevator();
  Intake intake = new Intake();
  Carriage carriage = new Carriage();

  //TODO: Change the LED length
  WS2812Driver rgbLED = new WS2812Driver(0, 10);

  //Commands 
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  ElevatorCommand elevatorCommand = new ElevatorCommand(elevator, 112);
  ElevatorUpCommand elevatorUpCommand = new ElevatorUpCommand(elevator);
  ElevatorDownCommand elevatorDownCommand = new ElevatorDownCommand(elevator);
  ElevatorHome elevatorHome = new ElevatorHome(elevator);
  CarriageCommand carriageCommand = new CarriageCommand(carriage, 15);
  IntakeCommand intakeCommand = new IntakeCommand(intake);
  ElevatorPOV elevatorPOV = new ElevatorPOV(operator, elevator, elevatorUpCommand, elevatorDownCommand, elevatorHome, elevatorCommand);

  public final static PowerDistribution pdh = new PowerDistribution();

  public RobotContainer() {
  
    // Configure the trigger bindings
    RobotState.setGamePiece(GamePiece.EMPTY);
    RobotState.setIntakeState(IntakeState.STOP);
    RobotState.setSwerve(SwerveState.REST);
    RobotState.setElevatorLevel(ElevatorLevel.ZERO);
    configureBindings();
  }

  private void configureBindings() {
    boolean a = RobotState.getTripping();
    swerveDrivetrain.setDefaultCommand(driveCommand);
    /*JoystickButton elevator1 = new JoystickButton(driver, 5);
    elevator1.whileTrue(elevatorUpCommand);

    JoystickButton elevator2 = new JoystickButton(driver,6);
    elevator2.whileTrue(elevatorDownCommand);

     JoystickButton elevator3 = new JoystickButton(operator,2);
   elevator3.whileTrue(new ElevatorCommand(elevator, 112));

   JoystickButton elevatorDown = new JoystickButton(operator,3);
   elevatorDown.whileTrue(new ElevatorCommand(elevator, 10));
   elevatorDown.whileFalse(new RunCommand(()-> elevator.stop()));*/

   JoystickButton carriage1 = new JoystickButton(operator, 10);
   carriage1.whileTrue(new RunCommand(()-> carriage.intakeUp(), carriage));
   carriage1.whileFalse(new RunCommand(()-> carriage.stop(), carriage));

   JoystickButton carriage2 = new JoystickButton(operator,9);
   carriage2.whileTrue(new RunCommand(()-> carriage.intakeDown(), carriage));
   carriage2.whileFalse(new RunCommand(()-> carriage.stop(), carriage));

   /*JoystickButton intake1 = new JoystickButton(operator, 6);
   intake1.whileTrue(new RunCommand(()-> intake.grabCone(), intake));
   intake1.whileFalse(new RunCommand(()-> intake.stop(), intake));

   JoystickButton intake2 = new JoystickButton(operator,5);
   intake2.whileTrue(new RunCommand(()-> intake.grabCube(), intake));
   intake2.whileFalse(new RunCommand(()-> intake.stop(), intake));
   swerveDrivetrain.setDefaultCommand(driveCommand);*/

   //TODO: Change this button
   /*JoystickButton encoderReset = new JoystickButton(operator, 8);
   encoderReset.onTrue(new RunCommand(() -> elevator.resetEncoder(), elevator));*/


   //GAME PIECE SELECTOR BUTTONS
   JoystickButton cubeButton = new JoystickButton(operator, 8);
   JoystickButton coneButton = new JoystickButton(operator, 7);
   coneButton.whileTrue(new RunCommand(()->RobotState.setCone()));
   cubeButton.whileTrue(new RunCommand(()->RobotState.setCube()));

   JoystickButton intakeButton = new JoystickButton(operator, 6);
   intakeButton.whileTrue(new RunCommand(()->RobotState.setIntaking()));
   intakeButton.whileFalse(new RunCommand(()-> RobotState.setIntakeIdle()));

   JoystickButton ejectButton = new JoystickButton(operator, 5);
   ejectButton.whileTrue(new RunCommand(()->RobotState.setEjecting()));
   ejectButton.whileFalse(new RunCommand(()-> RobotState.setIntakeIdle()));

   //Elevator Button Bindings
   intake.setDefaultCommand(intakeCommand);
   elevator.setDefaultCommand(elevatorPOV);
  
  }
  

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}