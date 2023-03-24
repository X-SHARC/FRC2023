package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

    double chargeStationMaxVel = 1.5;
    double chargeStationMaxAccel = 2;

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
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(0.5),
            new ConditionalCommand(
                new RunCommand(()-> elevator.setDistance(108), elevator).withTimeout(0.9),
                new RunCommand(()-> elevator.setDistance(105), elevator).withTimeout(0.9),
                RobotState::isCone
                ),
            new InstantCommand(()-> elevator.stop(), elevator),
            new ConditionalCommand(
                new RunCommand(()-> carriage.setDegrees(48), carriage).withTimeout(0.5),
                new RunCommand(()-> carriage.setDegrees(38), carriage).withTimeout(0.5),
                RobotState::isCone
            ),
            new InstantCommand(()-> carriage.stop(), carriage),
            new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(1),
            new InstantCommand(()-> carriage.stop(), carriage),
            new ElevatorHome(elevator).withTimeout(0.9),
            new InstantCommand(()-> elevator.stop(), elevator),
            new RunCommand(()-> carriage.setDegrees(7), carriage).withTimeout(1),
            new InstantCommand(()-> carriage.stop(), carriage)
            ),
            getControllerCommand(swerve, "LeftCube1", true, 3, 4).withTimeout(2.5),
            new RunCommand(()->carriage.setDegrees(100)).withTimeout(0.6),
            new InstantCommand(()-> carriage.stop(), carriage),
            new RunCommand(()->RobotState.setIntaking())
            .withTimeout(0.9),
            //.raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(1),
            new InstantCommand(()-> carriage.stop(), carriage),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            getControllerCommand(swerve, "LeftCube2", false, 3, 4).withTimeout(2.5),
            new RunCommand(()->RobotState.setEjecting())
            .withTimeout(0.9),
            //.raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new InstantCommand(()->RobotState.setIntakeIdle())
        );
    }

    public Command getLeftTwoCubeWithDock(Swerve swerve, Elevator elevator, Intake intake, Carriage carriage){
        RobotState.setGamePiece(RobotState.GamePiece.CUBE);
        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                new RunCommand(()-> carriage.setDegrees(30), carriage).withTimeout(0.55),
                new InstantCommand(()-> carriage.stop(), carriage), 
                new ConditionalCommand(
                  new RunCommand(()-> elevator.setDistance(108), elevator).withTimeout(0.8),
                  new RunCommand(()-> elevator.setDistance(104), elevator).withTimeout(0.7),
                  RobotState::isCone
                  ),
                new InstantCommand(()-> elevator.stop(), elevator),
                new ConditionalCommand(
                  new RunCommand(()-> carriage.setDegrees(48), carriage).withTimeout(0.4),
                  new RunCommand(()-> carriage.setDegrees(32), carriage).withTimeout(0.33),
                  RobotState::isCone
                ),
                new InstantCommand(()-> carriage.stop(), carriage),
                new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.3),
                new InstantCommand(()->RobotState.setIntakeIdle()),
                new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(0.5),
                new InstantCommand(()-> carriage.stop(), carriage),
                new ElevatorHome(elevator).withTimeout(0.7),
                new InstantCommand(()-> elevator.stop(), elevator),
                new RunCommand(()-> carriage.setDegrees(15), carriage).withTimeout(0.5),
                new InstantCommand(()-> carriage.stop(), carriage)
              ),
            Commands.parallel(
                new SequentialCommandGroup(
                    getControllerCommand(swerve, "LeftCube1", true, 3, 4).withTimeout(2.5),
                    new RunCommand(()->carriage.setDegrees(100)).withTimeout(0.9),
                    new InstantCommand(()-> carriage.stop(), carriage),
                    new RunCommand(()->RobotState.setIntaking())
                    .withTimeout(0.5)
                    .alongWith(new RunCommand(()->swerve.stopModules()).withTimeout(0.1)),
                    getControllerCommand(swerve, "LeftCube2", false, 3, 4).withTimeout(2.5)
                    .alongWith(
                        new SequentialCommandGroup(
                            new RunCommand(()->carriage.setDegrees(10), carriage).withTimeout(0.7),
                            new InstantCommand(()-> carriage.stop(), carriage),
                            new InstantCommand(()->RobotState.setIntakeIdle())
                        )
                    ),
                    new RunCommand(()->RobotState.setShooting()).withTimeout(0.35),
                    new InstantCommand(()->RobotState.setIntakeIdle()),
                    getControllerCommand(swerve, "LeftCubeDock", false, chargeStationMaxVel, chargeStationMaxAccel),
                    new RunCommand(()->swerve.stopModules()).withTimeout(1)
                ),
                new RunCommand(()->elevator.stop())
            )
            
        );
    }

    public Command getOneCubeAndBack(Swerve swerve, Elevator elevator, Carriage carriage){
        RobotState.setGamePiece(RobotState.GamePiece.CUBE);
        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                new ElevatorCommand(elevator, RobotState.getGamePiece() == GamePiece.CONE ? 112: 101).withTimeout(0.9)
                    .alongWith(new InstantCommand(()->carriage.setSetpoint(RobotState.getGamePiece() == GamePiece.CUBE ? 32:48))),
                new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
                new RunCommand(()->RobotState.setIntakeIdle()).withTimeout(0.01),
                new ElevatorHome(elevator).withTimeout(0.8)
                    .alongWith(new InstantCommand(()->carriage.setSetpoint(15))),
                new InstantCommand(()->elevator.stop())
            ),
            getControllerCommand(swerve, "1Cube&Back", true, chargeStationMaxVel, chargeStationMaxAccel),
            new RunCommand(()->swerve.stopModules()).withTimeout(1)
        );
    }


    public Command getLeft3Cube(Swerve swerve, Elevator elevator, Carriage carriage, Intake intake){
        RobotState.setGamePiece(RobotState.GamePiece.CUBE);

        return new SequentialCommandGroup(
            new SequentialCommandGroup(
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(0.5),
            new ConditionalCommand(
                new RunCommand(()-> elevator.setDistance(64), elevator).withTimeout(0.6),
                new RunCommand(()-> elevator.setDistance(70), elevator).withTimeout(0.6),
                RobotState::isCone
                ),
            new InstantCommand(()-> elevator.stop(), elevator),
            new ConditionalCommand(
                new RunCommand(()-> carriage.setDegrees(35), carriage).withTimeout(0.5),
                new RunCommand(()-> carriage.setDegrees(48), carriage).withTimeout(0.5),
                RobotState::isCone
            ),
            new InstantCommand(()-> carriage.stop(), carriage),
            new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.6),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(1),
            new InstantCommand(()-> carriage.stop(), carriage),
            new ElevatorHome(elevator).withTimeout(0.9),
            new InstantCommand(()-> elevator.stop(), elevator),
            new RunCommand(()-> carriage.setDegrees(7), carriage).withTimeout(1),
            new InstantCommand(()-> carriage.stop(), carriage)
            ),
            
            new SequentialCommandGroup(
            getControllerCommand(swerve, "LeftCube1", true, 3, 4).withTimeout(2.5),
            new RunCommand(()->carriage.setDegrees(100)).withTimeout(0.6),
            new InstantCommand(()-> carriage.stop(), carriage),
            new RunCommand(()->RobotState.setIntaking())
            .withTimeout(0.6),
            //.raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(1),
            new InstantCommand(()-> carriage.stop(), carriage)
            .withTimeout(0.6),
            //.raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new InstantCommand(()->RobotState.setIntakeIdle()),

            getControllerCommand(swerve, "LeftShoot2", false, 3, 4).withTimeout(2.8),
            new RunCommand(()->carriage.setDegrees(100)).withTimeout(0.6),
            new InstantCommand(()-> carriage.stop(), carriage),
            new RunCommand(()->RobotState.setIntaking())
            .withTimeout(0.6),
            //.raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(1),
            new InstantCommand(()-> carriage.stop(), carriage),
            new InstantCommand(()->RobotState.setIntakeIdle()),

            getControllerCommand(swerve, "LeftShoot3", false, 3, 4).withTimeout(2.5),
            new RunCommand(()->RobotState.setShooting())
            .withTimeout(0.6),
            //.raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new InstantCommand(()->RobotState.setIntakeIdle())
        ));
    }

    public Command getControllerCommand(Swerve swerve, String trajName, boolean isFirstTrajectory, double maxVel, double maxAccel) {
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajName, maxVel, maxAccel);
        swerve.addTrajectoryToField2d(trajectory);

        if(isFirstTrajectory) swerve.resetPoseEstimator(trajectory.getInitialHolonomicPose());

        return 
            new SequentialCommandGroup(
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
                .andThen(
                    new RunCommand(()->swerve.stopModules(), swerve).withTimeout(0.1)
                )
                );
    }

    
}
