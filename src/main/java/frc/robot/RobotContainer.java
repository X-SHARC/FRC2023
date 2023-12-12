package frc.robot;

import frc.robot.subsystems.Swerve;
import frc.robot.commands.Swerve.SwerveAntiDefense;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static XboxController driver = new XboxController(0);

  //Subsystems
  Swerve swerveDrivetrain = new Swerve(false);


  //Commands 
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  SwerveAntiDefense defense = new SwerveAntiDefense(swerveDrivetrain);

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    swerveDrivetrain.setDefaultCommand(driveCommand);
  }
  

  public Command getAutonomousCommand() {
    return null;

    }
  }