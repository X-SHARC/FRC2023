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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Elevator.ElevatorHome;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SharcTrajectory {
    //private PathPlannerTrajectory trajectory;
    //private PathPlannerState pathState;

    double chargeStationMaxVel = 1.5;
    double chargeStationMaxAccel = 2;

    PIDController xSpeedController = new PIDController(5.31, 0, 0);
    //y: 5.7
    PIDController ySpeedController = new PIDController(5.7, 0, 0);
    PIDController rotController = new PIDController(1.15, 0, 0);
    //1.27 pid, 1.431

    /*HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("intakeDown", new IntakeDown());*/
    
    public SharcTrajectory(){
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command getLeftTwoCube(Swerve swerve, Elevator elevator, Intake intake, Carriage carriage){
        RobotState.setGamePiece(RobotState.GamePiece.CUBE);

        return new SequentialCommandGroup(
            new SequentialCommandGroup(
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(0.4),
            new ConditionalCommand(
                new RunCommand(()-> elevator.setDistance(115), elevator).withTimeout(0.8),
                new RunCommand(()-> elevator.setDistance(108), elevator).withTimeout(0.7),
                RobotState::isCone
                ),
            new InstantCommand(()-> elevator.stop(), elevator),
            new ConditionalCommand(
                new RunCommand(()-> carriage.setDegrees(40), carriage).withTimeout(0.3),
                new RunCommand(()-> carriage.setDegrees(35), carriage).withTimeout(0.2),
                RobotState::isCone
            ),
            new InstantCommand(()-> carriage.stop(), carriage),
            new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.3),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(0.3),
            new InstantCommand(()-> carriage.stop(), carriage),
            new ElevatorHome(elevator).withTimeout(0.7),
            new InstantCommand(()-> elevator.stop(), elevator),
            new RunCommand(()-> carriage.setDegrees(7), carriage).withTimeout(0.2),
            new InstantCommand(()-> carriage.stop(), carriage)
            ),
            getControllerCommand(swerve, "LeftCube1", true, 4, 3).withTimeout(2.67),
            new RunCommand(()->carriage.setDegrees(105)).withTimeout(0.931),
            new InstantCommand(()-> carriage.stop(), carriage),
            new RunCommand(()->RobotState.setIntaking())
            .withTimeout(0.5),
            //.raceWith(new RunCommand(()->swerve.stopModules()).withTimeout(1)),
            new RunCommand(()-> carriage.setDegrees(25), carriage).withTimeout(1),
            new InstantCommand(()-> carriage.stop(), carriage),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            getControllerCommand(swerve, "LeftCube2", false, 4, 3).withTimeout(2.5)
            .alongWith(
                        new SequentialCommandGroup(
                            new RunCommand(()->carriage.setDegrees(10), carriage).withTimeout(0.55),
                            new InstantCommand(()-> carriage.stop(), carriage),
                            new InstantCommand(()->RobotState.setIntakeIdle())
                        )
                    ),
                    new RunCommand(()->RobotState.setEjecting()).withTimeout(0.35),
                    new InstantCommand(()->RobotState.setIntakeIdle())
        );
    }

    public Command getLeftTwoCubeWithDock(Swerve swerve, Elevator elevator, Intake intake, Carriage carriage){
        RobotState.setGamePiece(RobotState.GamePiece.CUBE);
        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                new RunCommand(()-> carriage.setDegrees(30), carriage).withTimeout(0.35),
                new InstantCommand(()-> carriage.stop(), carriage), 
                new ConditionalCommand(
                  new RunCommand(()-> elevator.setDistance(115), elevator).withTimeout(0.8),
                  new RunCommand(()-> elevator.setDistance(110), elevator).withTimeout(0.72),
                  RobotState::isCone
                  ),
                new InstantCommand(()-> elevator.stop(), elevator),
                new ConditionalCommand(
                  new RunCommand(()-> carriage.setDegrees(40), carriage).withTimeout(0.4),
                  new RunCommand(()-> carriage.setDegrees(35), carriage).withTimeout(0.21),
                  RobotState::isCone
                ),
                new InstantCommand(()-> carriage.stop(), carriage),
                new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.3),
                new InstantCommand(()->RobotState.setIntakeIdle()),
                new RunCommand(()-> carriage.setDegrees(23), carriage).withTimeout(0.2),
                new InstantCommand(()-> carriage.stop(), carriage),
                new ElevatorHome(elevator).withTimeout(0.75),
                new InstantCommand(()-> elevator.stop(), elevator),
                new RunCommand(()-> carriage.setDegrees(15), carriage).withTimeout(0.15),
                new InstantCommand(()-> carriage.stop(), carriage)
              ),
            Commands.parallel(
                new SequentialCommandGroup(
                    getControllerCommand(swerve, "LeftCube1", true, 4, 3).withTimeout(2.67),
                    new RunCommand(()->carriage.setDegrees(108)).withTimeout(0.941),
                    new InstantCommand(()-> carriage.stop(), carriage),
                    new RunCommand(()->RobotState.setIntaking())
                    .withTimeout(0.65)
                    .alongWith(new RunCommand(()->swerve.stopModules()).withTimeout(0.4)),
                    getControllerCommand(swerve, "LeftCube2", false, 4, 3).withTimeout(2.5)
                    .alongWith(
                        new SequentialCommandGroup(
                            new RunCommand(()->carriage.setDegrees(10), carriage).withTimeout(0.7),
                            new InstantCommand(()-> carriage.stop(), carriage),
                            new InstantCommand(()->RobotState.setIntakeIdle())
                        )
                    ),
                    new RunCommand(()->RobotState.setEjecting()).withTimeout(0.35)
                    .alongWith(new RunCommand(()->swerve.stopModules())).withTimeout(0.35),
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
                new RunCommand(()-> carriage.setDegrees(30), carriage).withTimeout(0.35),
                new InstantCommand(()-> carriage.stop(), carriage), 
                new ConditionalCommand(
                  new RunCommand(()-> elevator.setDistance(108), elevator).withTimeout(0.7),
                  new RunCommand(()-> elevator.setDistance(104), elevator).withTimeout(0.6),
                  RobotState::isCone
                  ),
                new InstantCommand(()-> elevator.stop(), elevator),
                new ConditionalCommand(
                  new RunCommand(()-> carriage.setDegrees(48), carriage).withTimeout(0.4),
                  new RunCommand(()-> carriage.setDegrees(32), carriage).withTimeout(0.1),
                  RobotState::isCone
                ),
                new InstantCommand(()-> carriage.stop(), carriage),
                new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.3),
                new InstantCommand(()->RobotState.setIntakeIdle()),
                new RunCommand(()-> carriage.setDegrees(23), carriage).withTimeout(0.2),
                new InstantCommand(()-> carriage.stop(), carriage),
                new ElevatorHome(elevator).withTimeout(0.75),
                new InstantCommand(()-> elevator.stop(), elevator),
                new RunCommand(()-> carriage.setDegrees(15), carriage).withTimeout(0.15),
                new InstantCommand(()-> carriage.stop(), carriage)
              ),
              new RunCommand(()-> swerve.drive(-0.2*Constants.Swerve.kMaxSpeed, 0,0, true, false), swerve).withTimeout(1.8)
              );
    }



    public Command getLeft3Cube(Swerve swerve, Elevator elevator, Intake intake, Carriage carriage){
        RobotState.setGamePiece(RobotState.GamePiece.CUBE);

        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                new RunCommand(()-> carriage.setDegrees(30), carriage).withTimeout(0.35),
                new InstantCommand(()-> carriage.stop(), carriage), 
                new ConditionalCommand(
                  new RunCommand(()-> elevator.setDistance(108), elevator).withTimeout(0.7),
                  new RunCommand(()-> elevator.setDistance(104), elevator).withTimeout(0.6),
                  RobotState::isCone
                  ),
                new InstantCommand(()-> elevator.stop(), elevator),
                new ConditionalCommand(
                  new RunCommand(()-> carriage.setDegrees(48), carriage).withTimeout(0.4),
                  new RunCommand(()-> carriage.setDegrees(32), carriage).withTimeout(0.1),
                  RobotState::isCone
                ),
                new InstantCommand(()-> carriage.stop(), carriage),
                new RunCommand(()-> RobotState.setEjecting()).withTimeout(0.3),
                new InstantCommand(()->RobotState.setIntakeIdle()),
                new RunCommand(()-> carriage.setDegrees(23), carriage).withTimeout(0.2),
                new InstantCommand(()-> carriage.stop(), carriage),
                new ElevatorHome(elevator).withTimeout(0.75),
                new InstantCommand(()-> elevator.stop(), elevator),
                new RunCommand(()-> carriage.setDegrees(15), carriage).withTimeout(0.15),
                new InstantCommand(()-> carriage.stop(), carriage)
              ),
            
            new SequentialCommandGroup(
            getControllerCommand(swerve, "LeftCube1", true, 4, 3).withTimeout(2.67),
            new RunCommand(()->carriage.setDegrees(100)).withTimeout(0.8),
            new InstantCommand(()-> carriage.stop(), carriage),
            new RunCommand(()->RobotState.setIntaking()).withTimeout(0.6),
            new RunCommand(()-> carriage.setDegrees(10), carriage).withTimeout(0.75),
            new InstantCommand(()-> carriage.stop(), carriage).withTimeout(0.6),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            getControllerCommand(swerve, "a", false, 4, 3).withTimeout(2.7)
            .alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(1.8),
                    new RunCommand(()->RobotState.setShooting()).withTimeout(1.2)        
                )),
            new InstantCommand(()->RobotState.setIntakeIdle()),
            getControllerCommand(swerve, "b", false, 4, 3).withTimeout(2.99),
            new RunCommand(()->carriage.setDegrees(100)).withTimeout(0.8),
            new InstantCommand(()-> carriage.stop(), carriage),
            new RunCommand(()->RobotState.setIntaking()).withTimeout(0.6),
            new RunCommand(()-> carriage.setDegrees(15), carriage).withTimeout(0.75),
            new InstantCommand(()-> carriage.stop(), carriage),
            new InstantCommand(()->RobotState.setIntakeIdle()),

            getControllerCommand(swerve, "c", false, 4, 3).withTimeout(2.9)
            .alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(2),
                    new RunCommand(()->RobotState.setShooting()).withTimeout(1.2)
                    )
                    ),
            new InstantCommand(()->RobotState.setIntakeIdle())
        ));
    }

    public Command getControllerCommand(Swerve swerve, String trajName, boolean isFirstTrajectory, double maxVel, double maxAccel) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajName, maxVel, maxAccel);

        if(isFirstTrajectory){
        
             
            if(DriverStation.getAlliance() == Alliance.Red){
                PathPlannerState initstate = trajectory.getInitialState();
                initstate = PathPlannerTrajectory.transformStateForAlliance(initstate, DriverStation.getAlliance());
                swerve.resetPoseEstimator(
                    new Pose2d(
                        new Translation2d(
                            initstate.poseMeters.getX(),
                            initstate.poseMeters.getY()
                        ),
                        initstate.holonomicRotation
                    )
            );
            }
            else if(DriverStation.getAlliance() == Alliance.Blue){
                swerve.resetPoseEstimator(
                    trajectory.getInitialHolonomicPose()
                );
            }
            else {
                swerve.resetPoseEstimator(
                    trajectory.getInitialHolonomicPose()
                );
            } 
        }

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
                true,
                swerve)
                .andThen(
                    new RunCommand(()->swerve.stopModules(), swerve).withTimeout(0.5)
                )
                );
    }

    
}
