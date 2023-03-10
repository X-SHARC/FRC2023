// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private double tx = 0;

  public Limelight() {

  }

  @Override
  public void periodic() {
    tx = LimelightHelpers.getTX(getName());
  }
}
