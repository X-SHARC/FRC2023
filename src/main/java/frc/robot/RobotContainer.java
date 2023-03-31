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
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static PS4Controller driver = new PS4Controller(0);
  //private final static Joystick driver = new Joystick(0);
  private final static Joystick operator = new Joystick(1);
  private final static PS4Controller operatorYedek = new PS4Controller(2);

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

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  SharcTrajectory trajCommands = new SharcTrajectory();
  

  //Autonomous Commands
  /*Command getLeft2Cube = trajCommands.getLeftTwoCube(swerveDrivetrain, elevator, intake, carriage);
  Command getLeft2CubeWithDock = trajCommands.getLeftTwoCubeWithDock(swerveDrivetrain,elevator,intake,carriage);
  Command getLeft3Cube = trajCommands.getLeft3Cube(swerveDrivetrain, elevator, intake, carriage);
  Command oneCubeTaxi = trajCommands.getOneCubeAndBack(swerveDrivetrain, elevator, carriage);
*/

  public RobotContainer() {
    m_chooser.addOption("2 Cubes + Dock", trajectoryGenerator.getLeftTwoCubeWithDock(swerveDrivetrain, elevator, intake, carriage));
    m_chooser.addOption("2 Cubes", trajectoryGenerator.getLeftTwoCube(swerveDrivetrain, elevator, intake, carriage));
    m_chooser.addOption("1 Cube and taxi", trajectoryGenerator.getOneCubeAndBack(swerveDrivetrain, elevator, carriage));
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
    elevatorhome.onTrue(elevatorHome); 

    JoystickButton carriageUp = new JoystickButton(operator, 3);
    carriageUp.whileTrue(new RunCommand(()-> carriage.intakeUp())); 
    carriageUp.onFalse(new InstantCommand(()->carriage.stop()));
  
   // denenecek + button atama kontrol
   JoystickButton carriageDown = new JoystickButton(operator, 5);
   carriageDown.whileTrue(new RunCommand(()-> carriage.intakeDown()));
   carriageDown.whileFalse(new RunCommand(()->carriage.stop()));

   JoystickButton carriageHome = new JoystickButton(operator, 4);
   carriageHome.whileTrue(new RunCommand(()->carriage.setDegrees(15)));
   carriageHome.whileFalse(new RunCommand(()->carriage.stop()));

   Trigger gyroReset = new Trigger(()->driver.getR3ButtonPressed());
   gyroReset.onTrue(new InstantCommand(()->swerveDrivetrain.resetPigeon()));


   Trigger hpStationTrigger = new Trigger(
    ()-> driver.getR2Axis()>0.5
   );

    hpStationTrigger.whileTrue(
      new SequentialCommandGroup(
        new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(0.65),
        new RunCommand(()-> elevator.setDistance(115), elevator).withTimeout(0.8),
        new InstantCommand(()-> elevator.stop(), elevator),
        new RunCommand(()-> carriage.setDegrees(81.5), carriage).withTimeout(0.77),
        new InstantCommand(()-> carriage.stop(), carriage),
        new RunCommand(()-> RobotState.setIntaking())
      )
    );

    hpStationTrigger.onFalse(
      new SequentialCommandGroup(
        new InstantCommand(()->RobotState.setIntakeIdle()),
        new RunCommand(()-> carriage.setDegrees(45), carriage).withTimeout(0.75),
        new InstantCommand(()-> carriage.stop(), carriage),
        new ElevatorHome(elevator).withTimeout(0.9),
        new InstantCommand(()-> elevator.stop(), elevator),
        new RunCommand(()-> carriage.setDegrees(7), carriage).withTimeout(0.85),
        new InstantCommand(()-> carriage.stop(), carriage)
      )
    );

   JoystickButton secondLevel = new JoystickButton(driver, 2);
    secondLevel.whileTrue(
      new SequentialCommandGroup(
        new RunCommand(()-> carriage.setDegrees(35), carriage).withTimeout(0.65),
        new ConditionalCommand(
          new RunCommand(()-> elevator.setDistance(64), elevator).withTimeout(0.6),
          new RunCommand(()-> elevator.setDistance(70), elevator).withTimeout(0.6),
          RobotState::isCone
          ),
        new InstantCommand(()-> elevator.stop(), elevator),
        new ConditionalCommand(
          new RunCommand(()-> carriage.setDegrees(35), carriage).withTimeout(0.65),
          new RunCommand(()-> carriage.setDegrees(48), carriage).withTimeout(0.65),
          RobotState::isCone
        ),
        new InstantCommand(()-> carriage.stop(), carriage),
        new ConditionalCommand(
          new RunCommand(()-> elevator.setDistance(64), elevator).withTimeout(0.6),
          new RunCommand(()-> elevator.setDistance(70), elevator).withTimeout(0.6),
          RobotState::isCone
          ),
          new InstantCommand(()-> carriage.stop(), carriage),
          new InstantCommand(()-> elevator.stop(), elevator)
        )
    );
    secondLevel.onFalse(
      new SequentialCommandGroup(
        new InstantCommand(()-> carriage.stop(), carriage),
        new InstantCommand(()-> elevator.stop(), elevator),
        new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
        new InstantCommand(()->RobotState.setIntakeIdle()),
        new ElevatorHome(elevator).withTimeout(0.75),
        new InstantCommand(()-> elevator.stop(), elevator),
        new RunCommand(()-> carriage.setDegrees(7), carriage).withTimeout(0.85),
        new InstantCommand(()-> carriage.stop(), carriage)
      )
    );

   /* 
   secondLevel.onTrue(
    new SequentialCommandGroup(
      //new RunCommand(()->carriage.setSetpoint(15), carriage).withTimeout(0.5),
      new ConditionalCommand(
        new RunCommand(()-> elevator.setDistance(60), elevator).withTimeout(1),
        new RunCommand(()-> elevator.setDistance(62), elevator).withTimeout(1),
        RobotState::isCone
      )
        .alongWith(
          new ConditionalCommand(
            new RunCommand(()-> carriage.setDegrees(38), carriage).withTimeout(0.5),
            new RunCommand(()-> carriage.setDegrees(48), carriage).withTimeout(0.5),
            RobotState::isCone
          )),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
      new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
      new ElevatorHome(elevator).withTimeout(1.8)
      .alongWith(new RunCommand(()-> carriage.setDegrees(10), carriage).withTimeout(0.5))
      ).beforeStarting(
        new SequentialCommandGroup(
          new RunCommand(()-> carriage.setDegrees(20), carriage).withTimeout(0.5),
          new InstantCommand(()-> carriage.stop(), carriage),
          new WaitCommand(0.4)
        )
        )
   ); */

   // secondLevel.onFalse(new RunCommand(()->elevator.stop()));

   JoystickButton thirdLevel = new JoystickButton(driver, 4);
   thirdLevel.whileTrue(
    new SequentialCommandGroup(
      new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(0.5),
      new ConditionalCommand(
        new RunCommand(()-> elevator.setDistance(115), elevator).withTimeout(0.8),
        new RunCommand(()-> elevator.setDistance(108), elevator).withTimeout(0.7),
        RobotState::isCone
        ),
      new InstantCommand(()-> elevator.stop(), elevator),
      new ConditionalCommand(
        new RunCommand(()-> carriage.setDegrees(40), carriage).withTimeout(0.45),
        new RunCommand(()-> carriage.setDegrees(35), carriage).withTimeout(0.4),
        RobotState::isCone
      ),
      new InstantCommand(()-> carriage.stop(), carriage),
      new ConditionalCommand(
        new RunCommand(()-> elevator.setDistance(115), elevator).withTimeout(0.8),
        new RunCommand(()-> elevator.setDistance(108), elevator).withTimeout(0.7),
        RobotState::isCone
        )
    )
   );

   thirdLevel.onFalse(
    new SequentialCommandGroup(
      new InstantCommand(()-> elevator.stop(), elevator),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
      new InstantCommand(()->RobotState.setIntakeIdle()),
      new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(0.37),
      new InstantCommand(()-> carriage.stop(), carriage),
      new ElevatorHome(elevator).withTimeout(0.8),
      new InstantCommand(()-> elevator.stop(), elevator),
      new RunCommand(()-> carriage.setDegrees(7), carriage).withTimeout(1.15),
      new InstantCommand(()-> carriage.stop(), carriage)
    )
   );
   /*
   thirdLevel.onTrue(
    new SequentialCommandGroup(
      //new RunCommand(()->carriage.setSetpoint(15), carriage).withTimeout(0.5),
      new ConditionalCommand(
        new ElevatorCommand(elevator, 114).withTimeout(0.9),
        new ElevatorCommand(elevator, 110).withTimeout(0.9),
        RobotState::isCone 
      )
      .alongWith(
        new ConditionalCommand(
          new InstantCommand(()->carriage.setSetpoint(48)),
          new InstantCommand(()->carriage.setSetpoint(38)),
          RobotState::isCone
        )),
      new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
      new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
      new ElevatorHome(elevator).withTimeout(1.8)
      .alongWith(new InstantCommand(()->carriage.setSetpoint(15)))
    )
    .beforeStarting(
      Commands.parallel(
       new InstantCommand(()->carriage.setSetpoint(20)),
       new WaitCommand(0.4)
       )
      )
   );
    */

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
   //encoderReset.onTrue(new RunCommand(() -> elevator.resetEncoder(), elevator));

   JoystickButton intakeButton = new JoystickButton(operator, 2);
   intakeButton.whileTrue(
    new SequentialCommandGroup(
      new InstantCommand(()-> carriage.stop()),
      new RunCommand(()->RobotState.setIntaking())
    )
    .beforeStarting(new RunCommand(()->carriage.setDegrees(RobotState.getGamePiece()==GamePiece.CONE? 82:100)).withTimeout(0.55))
    );

   intakeButton.whileFalse(
    new SequentialCommandGroup(
      new InstantCommand(()-> RobotState.setIntakeIdle()),
      new RunCommand(()->carriage.setDegrees(12), carriage).withTimeout(0.5),
      new InstantCommand(()-> carriage.stop(), carriage)
    )
    );

   JoystickButton ejectButton = new JoystickButton(operator, 1);
   ejectButton.whileTrue(
    new SequentialCommandGroup(
      new InstantCommand(()-> carriage.stop()),
      new RunCommand(()->RobotState.setEjecting())
    )
   .beforeStarting(
    new RunCommand(()->carriage.setDegrees(RobotState.getGamePiece() == GamePiece.CONE ? 50:75 )).withTimeout(0.5)
   ));

   ejectButton.whileFalse(
    new SequentialCommandGroup(
      new InstantCommand(()-> RobotState.setIntakeIdle()),
      new RunCommand(()->carriage.setDegrees(12)).withTimeout(0.5),
      new InstantCommand(()-> carriage.stop())
   ));

    JoystickButton shootButton = new JoystickButton(operator, 6);
    shootButton.whileTrue(new InstantCommand(()->RobotState.setShooting()));
    shootButton.whileFalse(
      new RunCommand(()-> RobotState.setIntakeIdle()));

   intake.setDefaultCommand(intakeCommand);
  }
  

  public Command getAutonomousCommand() {
    //return trajectoryGenerator.getLeftTwoCube(swerveDrivetrain, elevator, intake, carriage);
    return m_chooser.getSelected();
  }
} 