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

//Drive Motor ID's

public static final int Front_Left_DriveID = 1;
public static final int Rear_Left_DriveID =2;
public static final int Front_Right_DriveID =3;
public static final int Rear_Right_DriveID =4;

//Max Speeds

public static final double Max_Drive_Speed = 0.5;
public static final double Max_Rotation_Speeed = 0.5;

public static final double newcode = 5;

  }//////////////////////////////////////


public static final class DrivePID {

  public static final double DrivekP = .5;
  public static final double DrivekI = .1;
  public static final double DrivekD = 0.0;
   public static final double DriveFF = 0.0;
   public static final double tolerance = 0;



}




  
}
