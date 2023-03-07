// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Carriage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CarriageHome extends SequentialCommandGroup {
  Carriage carriage;
  /** Creates a new CarriageHome. */
  public CarriageHome(Carriage carriage) {
    this.carriage = carriage;


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // degerler ayarli degil
      new CarriageCommand(carriage, 5).withTimeout(0.8),
      new RunCommand(()->carriage.carriageHome(0.20), carriage).until(carriage::getCarriageHome),
      new RunCommand(()-> carriage.stop(), carriage),
      new RunCommand(()-> this.end(true))
    );
  }
}
