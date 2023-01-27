// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Slider;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.Autos;
import frc.robot.commands.SliderBackwardCommand;
import frc.robot.commands.SliderForwardCommand;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  //Subsystems
  Swerve swerveDrivetrain = new Swerve(true);
  Slider slider = new Slider();

  //Commands 
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  SliderBackwardCommand sliderBackwardCommand = new SliderBackwardCommand(slider);
  SliderForwardCommand sliderForwardCommand = new SliderForwardCommand(slider);


      public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    swerveDrivetrain.setDefaultCommand(driveCommand);

    JoystickButton[] sliderButtons = {
      new JoystickButton(operator, 1),
      new JoystickButton(operator, 2)
    };

    sliderButtons[0]
    .whileTrue(new RunCommand(()-> slider.sliderForward(), slider))
    .whileFalse(new RunCommand(()-> slider.stop(), slider));
  

    sliderButtons[1]
   .whileTrue(new RunCommand(()-> slider.sliderBackwards(), slider))
   .whileFalse(new RunCommand(()-> slider.stop(), slider));

  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto();
  }
}