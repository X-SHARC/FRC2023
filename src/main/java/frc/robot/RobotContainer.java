// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.CarriageCommand;
import frc.robot.commands.CarriageHome;
import frc.robot.commands.ConeToLevelThree;
import frc.robot.commands.ConeToLevelTwo;
import frc.robot.commands.CubeToLevelThree;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TwoToThree;
import frc.robot.commands.Autonomous.AutoAlign;
import frc.robot.commands.CubeToLevelTwo;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ElevatorDownCommand;
import frc.robot.commands.Elevator.ElevatorHome;
import frc.robot.commands.Elevator.ElevatorUpCommand;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.commands.Swerve.TurnToAngle;
import frc.robot.lib.drivers.WS2812Driver;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  //TODO: Change the LED length
  WS2812Driver rgbLED = new WS2812Driver(0, 13);

  //Commands 
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  ElevatorCommand elevatorCommand = new ElevatorCommand(elevator, 75);
  ElevatorUpCommand elevatorUpCommand = new ElevatorUpCommand(elevator);
  ElevatorDownCommand elevatorDownCommand = new ElevatorDownCommand(elevator);
  ElevatorHome elevatorHome = new ElevatorHome(elevator);
  CarriageHome carriageHomeCommand = new CarriageHome(carriage);
  CarriageCommand carriageCommand = new CarriageCommand(carriage, 67);
  IntakeCommand intakeCommand = new IntakeCommand(intake,operator);

  PowerDistribution pdh = new PowerDistribution();

  CubeToLevelTwo secondLevelcube = new CubeToLevelTwo(elevator, intake, carriage);
  ConeToLevelTwo secondLevelcone =  new ConeToLevelTwo(elevator, intake, carriage);
  CubeToLevelThree thirdLevelCube = new CubeToLevelThree(elevator, intake, carriage);
  ConeToLevelThree thirdLevelCone = new ConeToLevelThree(elevator, intake, carriage);
  TwoToThree twoToThree = new TwoToThree(elevator, intake, carriage);

  AutoAlign autoAlign = new AutoAlign(swerveDrivetrain);

  TurnToAngle turnToAngle = new TurnToAngle(swerveDrivetrain, 15);

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

    // button atama kontrol + denenecek 
    /*JoystickButton elevatorpid = new JoystickButton(driver,6);
    elevatorpid.whileTrue(elevatorCommand);*/

    JoystickButton elevatorhome = new JoystickButton(driver,7);
    elevatorhome.whileTrue(elevatorHome); 


    //TODO: add carriage default command: pid, home, limitations
   //BU COMMENTLERİ BURAK VE AYSU'YA SORMADAN AÇMAYIN
    /*
   JoystickButton carriage1 = new JoystickButton(operator, 10);
   //BU COMMENTLERİ BURAK VE AYSU'YA VE ELA'YA SORMADAN AÇMAYIN
   carriage1.whileTrue(new RunCommand(()-> carriage.intakeUp(), carriage));
   //BU COMMENTLERİ BURAK VE AYSU'YA SORMADAN AÇgiMAYIN
   carriage1.whileFalse(new RunCommand(()-> carriage.stop(), carriage));
   //BU COMMENTLERİ BURAK VE AYSU'YA SORMADAN AÇMAYIN
   JoystickButton carriage2 = new JoystickButton(operator, 9);
   //BU COMMENTLERİ BURAK VE AYSU'YA SORMADAN AÇMAYIN
   carriage2.whileTrue(new RunCommand(()-> carriage.intakeDown(), carriage));
   //BU COMMENTLERİ BURAK VE AYSU'YA SORMADAN AÇMAYIN
   carriage2.whileFalse(new RunCommand(()-> carriage.stop(), carriage));
   //BU COMMENTLERİ BURAK VE AYSU'YA SORMADAN AÇMAYIN
    */

    JoystickButton carriagepid = new JoystickButton(operator, 5);
   carriagepid.whileTrue(new InstantCommand(()-> carriage.setSetpoint(67))); 

   /*
   JoystickButton carriagereset = new JoystickButton(operator, 6);
   carriagereset.onTrue(new RunCommand(() -> carriage.resetCarriageEncoder(), carriage));
    */
  
   // denenecek + button atama kontrol
   JoystickButton carriagecommand = new JoystickButton(operator, 3);
   carriagecommand.onTrue(new InstantCommand(()-> carriage.setSetpoint(5)));




   JoystickButton secondLevelcube = new JoystickButton(driver, 1);
   secondLevelcube.onTrue(new CubeToLevelTwo(elevator, intake, carriage));
   secondLevelcube.onFalse(new RunCommand(()->elevator.stop())
   .alongWith(new RunCommand(()->carriage.stop(),carriage)));

   JoystickButton secondLevelcone = new JoystickButton(driver, 2);
   secondLevelcone.onTrue(new ConeToLevelTwo(elevator, intake, carriage));
   secondLevelcone.onFalse(new RunCommand(()->elevator.stop())
   .alongWith(new RunCommand(()->carriage.stop(),carriage)));

   JoystickButton thirdLevelcube = new JoystickButton(driver, 3);
   thirdLevelcube.onTrue(new CubeToLevelThree(elevator, intake, carriage));
   thirdLevelcube.onFalse(new RunCommand(()->elevator.stop())
   .alongWith(new RunCommand(()->carriage.stop(),carriage)));
 
   JoystickButton thirdLevelcone = new JoystickButton(driver, 4);
   thirdLevelcone.onTrue(new ConeToLevelThree(elevator, intake, carriage));
   thirdLevelcone.onFalse(new RunCommand(()->elevator.stop())
   .alongWith(new RunCommand(()->carriage.stop(),carriage)));

   JoystickButton turn = new JoystickButton(driver, 5);
   turn.whileTrue(autoAlign);


    /* 
   JoystickButton carriageButton = new JoystickButton(operator, 1);
   carriageButton.whileTrue(carriageCommand);

   JoystickButton resetCarriageButton = new JoystickButton(operator, 2);
   resetCarriageButton.onTrue(new RunCommand(()-> carriage.resetCarriageEncoder()));
  */

   //TODO: Change this button
   JoystickButton encoderReset = new JoystickButton(operator, 12);
   encoderReset.onTrue(new RunCommand(() -> elevator.resetEncoder(), elevator));

   JoystickButton intakeButton = new JoystickButton(operator, 2);
   intakeButton.whileTrue(new RunCommand(()->RobotState.setIntaking()));
   intakeButton.whileFalse(new RunCommand(()-> RobotState.setIntakeIdle()));

   JoystickButton ejectButton = new JoystickButton(operator, 1);
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