package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class SharcTrajectory {
    //private PathPlannerTrajectory trajectory;
    //private PathPlannerState pathState;

    public SharcTrajectory(){
        //trajectory = PathPlanner.loadPath("Cube&Dock", 3, 4);
        //pathState = (PathPlannerState) trajectory.sample(1);
    }

    PIDController xSpeedController = new PIDController(0, 0, 0);
    PIDController ySpeedController = new PIDController(0, 0, 0);
    PIDController rotController = new PIDController(0, 0, 0);

    public Command getControllerCommand(Swerve swerve, String trajName) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajName, 3, 4);
        return new SequentialCommandGroup(
            new InstantCommand(()->{
                swerve.resetOdometry(trajectory.getInitialHolonomicPose());
            }),
            new PPSwerveControllerCommand(
                trajectory,
                swerve::getPose,
                Constants.Swerve.kinematics,
                xSpeedController,
                ySpeedController,
                rotController,
                swerve::setClosedLoopStates,
                true,
                swerve)
        );
    }

    
}
