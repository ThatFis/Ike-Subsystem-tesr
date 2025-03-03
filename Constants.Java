// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  
    public static final int BallMotorI = 5; // CAN ID for intake ball
    public static final int BallMotorP = 6;  //CAN ID TO INTAKE BAL
    public static final int CoralMotorL = 7; // CAN ID fOR CORAL OUTTAKE   
    public static final int CoralMotorR = 8; // CAN ID FOR CORAL OUTAKE
    public static final int LiftMotor = 9; // CAN ID FOR elevator lift

   // public static final int ballintakeID = 17;
    
  
  }
}
