// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.lib.vision.VisionTarget;
import frc.robot.subsystems.Swerve;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  Swerve swerve;
  //0.0075
  PIDController sidewaysController = new PIDController(0.016, 0, 0);
  PIDController rotationController = new PIDController(0.035, 0, 0);
  PIDController distanceController = new PIDController(0.0165, 0, 0);

  VisionTarget lowerCone = new VisionTarget(59, 12, 20);

  public AutoAlign(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    sidewaysController.setTolerance(1);
    rotationController.setTolerance(1);
    rotationController.setTolerance(2);
  }

  @Override
  public void initialize() {
    LimelightHelpers.setCameraMode_Processor("limelight");


  }

  @Override
  public void execute() {
    double rotation = Constants.Swerve.kMaxAngularSpeed*rotationController.calculate(swerve.getGyroDouble(), 0);
    lowerCone.updatePitch(LimelightHelpers.getTY("limelight"));
    double output = Constants.Swerve.kMaxSpeed * 1. * sidewaysController.calculate(LimelightHelpers.getTX("limelight"));
    double distance  = lowerCone.getDistance();
    //systematic error is 13 centimeters
    
    if(Math.abs(swerve.getGyroDouble())>=3){
      swerve.drive(0, 0, rotation, false, true);
    }
    else if(Math.abs(LimelightHelpers.getTX("limelight"))>=4){
      swerve.drive(0, output, rotation, false,true);
    }
    else{ //Changed the setpoint
      double distanceOutput = -distanceController.calculate(distance, 60)*Constants.Swerve.kMaxSpeed;
      swerve.drive(distanceOutput, output, rotation, false,true);

    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
    LimelightHelpers.setCameraMode_Driver("limelight");
  }

  @Override
  public boolean isFinished() {
    return sidewaysController.atSetpoint() && distanceController.atSetpoint();
  }
}
