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

    PIDController xSpeedController = new PIDController(5.7, 0, 0);
    PIDController ySpeedController = new PIDController(5.7, 0, 0);
    PIDController rotController = new PIDController(1.5, 0, 0);
    
    public SharcTrajectory(){
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        //trajectory = PathPlanner.loadPath("Cube&Dock", 3, 4);
        //pathState = (PathPlannerState) trajectory.sample(1);
        //rotController.enableContinuousInput(-Math.PI, Math.PI);
    }


    public Command getControllerCommand(Swerve swerve, String trajName) {
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajName, 3, 4);
        swerve.addTrajectoryToField2d(trajectory);
        swerve.resetPoseEstimator(trajectory.getInitialHolonomicPose());
        return new SequentialCommandGroup(
            new PPSwerveControllerCommand(
                trajectory,
                swerve::getPose,
                Constants.Swerve.kinematics,
                xSpeedController,
                ySpeedController,
                rotController,
                swerve::setClosedLoopStates,
                false,
                swerve)
        );
    }

    
}
