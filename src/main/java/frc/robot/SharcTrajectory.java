package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState.GamePiece;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ElevatorHome;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.controller.PIDController;

public class SharcTrajectory {
    //private PathPlannerTrajectory trajectory;
    //private PathPlannerState pathState;

    PIDController xSpeedController = new PIDController(5.7, 0, 0);
    PIDController ySpeedController = new PIDController(5.7, 0, 0);
    PIDController rotController = new PIDController(1.27, 0, 0);
    
    public SharcTrajectory(){
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        //trajectory = PathPlanner.loadPath("Cube&Dock", 3, 4);
        //pathState = (PathPlannerState) trajectory.sample(1);
        //rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command getLeftTwoCube(Swerve swerve, Elevator elevator, Intake intake, Carriage carriage){
        RobotState.setGamePiece(RobotState.GamePiece.CUBE);

        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                new ElevatorCommand(elevator, RobotState.getGamePiece() == GamePiece.CONE ? 112: 101).withTimeout(0.9)
                    .alongWith(new InstantCommand(()->carriage.setSetpoint(RobotState.getGamePiece() == GamePiece.CUBE ? 32:48))),
                new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
                new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
                new ElevatorHome(elevator).withTimeout(0.8)
                    .alongWith(new InstantCommand(()->carriage.setSetpoint(15)))
            ),
            getControllerCommand(swerve, "LeftCube1", true).withTimeout(2.5),
            new InstantCommand(()->carriage.setSetpoint(100)),
            new RunCommand(()->RobotState.setIntaking())
            .withTimeout(0.9).raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new InstantCommand(()->carriage.setSetpoint(10)),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            getControllerCommand(swerve, "LeftCube2", false).withTimeout(2.5),
            new RunCommand(()->RobotState.setEjecting()).withTimeout(0.6)
            .withTimeout(0.9).raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new InstantCommand(()->RobotState.setIntakeIdle())
        );
    }

    public Command getLeftTwoCubeWithDock(Swerve swerve, Elevator elevator, Intake intake, Carriage carriage){
        RobotState.setGamePiece(RobotState.GamePiece.CUBE);
        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                new ElevatorCommand(elevator, RobotState.getGamePiece() == GamePiece.CONE ? 112: 101).withTimeout(0.9)
                    .alongWith(new InstantCommand(()->carriage.setSetpoint(RobotState.getGamePiece() == GamePiece.CUBE ? 32:48))),
                new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
                new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
                new ElevatorHome(elevator).withTimeout(0.8)
                    .alongWith(new InstantCommand(()->carriage.setSetpoint(15)))
            ),
            getControllerCommand(swerve, "LeftCube1", true).withTimeout(2.5),
            new InstantCommand(()->carriage.setSetpoint(100)),
            new RunCommand(()->RobotState.setIntaking())
            .withTimeout(0.9),
            //.raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new InstantCommand(()->carriage.setSetpoint(10)),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            getControllerCommand(swerve, "LeftCube2", false).withTimeout(2.5),
            new RunCommand(()->RobotState.setEjecting()).withTimeout(0.6)
            .withTimeout(0.9),
            //.raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            getControllerCommand(swerve, "LeftCubeDock", false)
        );
    }

    public Command getControllerCommand(Swerve swerve, String trajName, boolean isFirstTrajectory) {
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajName, 3, 4);
        swerve.addTrajectoryToField2d(trajectory);

        if(isFirstTrajectory) swerve.resetPoseEstimator(trajectory.getInitialHolonomicPose());

        return 
            new PPSwerveControllerCommand(
                trajectory,
                swerve::getPose,
                Constants.Swerve.kinematics,
                xSpeedController,
                ySpeedController,
                rotController,
                swerve::setClosedLoopStates,
                false,
                swerve);
    }

    
}
