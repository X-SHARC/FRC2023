package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class Trajectory {
    PathPlannerTrajectory firstTrial = PathPlanner.loadPath("Cube&Dock", 3, 4);
    PathPlannerState pathState = (PathPlannerState) firstTrial.sample(1);

    PIDController xSpeedController = new PIDController(0, 0, 0);
    PIDController ySpeedController = new PIDController(0, 0, 0);
    PIDController rotController = new PIDController(0, 0, 0);

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    private static Command getControllerCommand(Trajectory trajectory, Swerve swerve, PIDController x_pid, PIDController y_pid, ProfiledPIDController thetaController) {
        return null;
    }

    
}
