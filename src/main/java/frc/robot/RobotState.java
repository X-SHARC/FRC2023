// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class RobotState {

    private static RobotState robotState;
    public enum GamePiece{
        CONE,
        CUBE, 
        EMPTY
    }
    public enum ElevatorLevel{
        ZERO,
        MIDROW,
        TOPROW
    }
    public enum IntakeState{
        INTAKING,
        EJECTING,
        IDLE,
        STOP
    }

    public enum CarriageState{
        AUTO,
        MANUAL,
    }

    public enum SwerveState{
        MOVING,
        REST
    }

    public static IntakeState currentIntakeState = IntakeState.STOP;
    public static GamePiece currentGamePiece = GamePiece.EMPTY;
    public static ElevatorLevel currentElevatorLevel = ElevatorLevel.ZERO;
    public static SwerveState currentSwerveState = SwerveState.REST;
    public static CarriageState currentCarriageState = CarriageState.AUTO;
    public static boolean isElevated = false;
    public static boolean isTripping = false;
    public static boolean isCarriageHome = false;
    public static boolean isCarriageEncoderAlive = false;
    
    private RobotState(){
        //reset(); 
    }

    public static boolean isTeleop(){
        return edu.wpi.first.wpilibj.RobotState.isTeleop();
    }

    public static boolean isBlueAlliance(){
        return DriverStation.getAlliance() == Alliance.Blue;
    }

    public static boolean isRedAlliance(){
        return DriverStation.getAlliance() == Alliance.Red;
    }

    public static void setCarriageEncoder(boolean alive){
        isCarriageEncoderAlive = alive;
    }

    public static boolean isCarriageEncoderAlive(){
        return isCarriageEncoderAlive;
    }
    
    public static void setGamePiece(GamePiece gamePiece){
        currentGamePiece = gamePiece;
    }

    public static void setCone(){
        if(currentGamePiece != GamePiece.CONE){
            currentGamePiece = GamePiece.CONE;
        }
    }

    public static void setIntaking(){
        if(currentIntakeState!= IntakeState.INTAKING){
            currentIntakeState = IntakeState.INTAKING;
        }
    }

    public static void setCarriageAutonomous(){
        currentCarriageState = CarriageState.AUTO;
    }
    
    public static void setCarriageManual(){
        currentCarriageState = CarriageState.MANUAL;
    }

    public static CarriageState getCarriage(){
        return currentCarriageState;
    }

    public static void setEjecting(){
        if(currentIntakeState != IntakeState.EJECTING){
            currentIntakeState = IntakeState.EJECTING;
        }
    }

    public static void setCube(){
        if(currentGamePiece != GamePiece.CUBE){
            currentGamePiece = GamePiece.CUBE;
        }
    }

    public static void setIntakeState(IntakeState intakeState){
        currentIntakeState = intakeState;
    }

    public static void setElevated(boolean elevated){
        isElevated = elevated;
    }

    public static void setSwerve(SwerveState swerveState){
        currentSwerveState = swerveState;
    }

    public static void setIntakeIdle(){
        if(currentIntakeState != IntakeState.IDLE){
            currentIntakeState = IntakeState.IDLE;
        }
    }

    public static void setTripping(boolean m_isTripping){
        if(isTripping != m_isTripping){
            isTripping = m_isTripping;
        }
    }

    public static void setElevatorLevel(ElevatorLevel elevatorLevel){
        currentElevatorLevel = elevatorLevel;
    }

    public static GamePiece getGamePiece(){
        return currentGamePiece;
    }

    public static boolean isCone(){
        return getGamePiece() == GamePiece.CONE;
    }

    public static IntakeState getIntaking(){
        return currentIntakeState;
    }

    public static boolean isElevated(){
        return isElevated;
    }

    public static boolean getTripping(){
        return isTripping;
    }

    public static ElevatorLevel getElevatorLevel(){
        return currentElevatorLevel;
    }

    public static SwerveState getSwerveState(){
        return currentSwerveState;
    }

    public static void setCarriage(boolean home){
        isCarriageHome = home;
    }

    public static boolean canElevatorMove(){
        return !isCarriageHome;
    }

    public static int getElevatorLevelInt(){
       switch(currentElevatorLevel){
              case ZERO:
                 return 1;
              case MIDROW:
                 return 2;
              case TOPROW:
                 return 3;
              default:
                 return 0;
       }
    }

    public static void reset(){
        isCarriageHome = false;
        currentGamePiece = null;
        currentElevatorLevel = null;
        currentIntakeState = IntakeState.IDLE;
        isElevated = false;
        isTripping = false;
    }

    public static RobotState getInstance(){
        if(robotState == null){
            robotState = new RobotState();
        }
        return robotState;
    }



}
