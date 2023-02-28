// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotState {

    public static  RobotState robotState;
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

    public enum SwerveState{
        MOVING,
        REST
    }

    public static IntakeState currentIntakeState;
    public static GamePiece currentGamePiece = GamePiece.EMPTY;
    public static ElevatorLevel currentElevatorLevel;
    public static SwerveState currentSwerveState;
    public static boolean isElevating;
    public static boolean isTripping;
    
    private RobotState(){
        reset(); 
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

    public static void setElevating(boolean elevating){
        isElevating = elevating;
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

    public static IntakeState getIntaking(){
        return currentIntakeState;
    }

    public static boolean getElevating(){
        return isElevating;
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
        currentGamePiece = null;
        currentElevatorLevel = null;
        currentIntakeState = IntakeState.IDLE;
        isElevating = false;
        isTripping = false;
    }

    public static RobotState getInstance(){
        if(robotState == null){
            robotState = new RobotState();
        }
        return robotState;
    }



}
