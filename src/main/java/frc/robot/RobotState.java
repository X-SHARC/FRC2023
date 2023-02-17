// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotState {
    public static  RobotState robotState;
    public enum GamePiece{
        CONE,
        CARGO
    }
    public enum ElevatorLevel{
        LEVEL_1,
        LEVEL_2,
        LEVEL_3
    }

    public static GamePiece currentGamePiece;
    public static ElevatorLevel currentElevatorLevel;

    public static boolean isIntaking;
    public static boolean isElevating;
    public static boolean isTripping;
    
    public RobotState(){

    }

    public static void setGamePiece(GamePiece gamePiece){
        currentGamePiece = gamePiece;
    }

    public static void setCone(){
        currentGamePiece = GamePiece.CONE;
    }

    public static void setCargo(){
        currentGamePiece = GamePiece.CARGO;
    }

    public static void setIntaking(boolean isIntaking){
        RobotState.isIntaking = isIntaking;
    }

    public static void setElevating(boolean isElevating){
        RobotState.isElevating = isElevating;
    }

    public static void setTripping(boolean isTripping){
        RobotState.isTripping = isTripping;
    }

    public static void setElevatorLevel(ElevatorLevel elevatorLevel){
        currentElevatorLevel = elevatorLevel;
    }

    public static GamePiece getGamePiece(){
        return currentGamePiece;
    }

    public static boolean getIntaking(){
        return isIntaking;
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

    public static int getElevatorLevelInt(){
       switch(currentElevatorLevel){
              case LEVEL_1:
                 return 1;
              case LEVEL_2:
                 return 2;
              case LEVEL_3:
                 return 3;
              default:
                 return 0;
       }
    }

    public static void reset(){
        currentGamePiece = null;
        currentElevatorLevel = null;
        isIntaking = false;
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
