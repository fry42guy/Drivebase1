// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  
  public final WPI_TalonFX FrontLeft;
  public final WPI_TalonFX RearLeft;
  public final WPI_TalonFX FrontRight;
  public final WPI_TalonFX RearRight;

  public final MotorControllerGroup LeftDrive;
  public final MotorControllerGroup RightDrive;

  public final DifferentialDrive m_Drive;



  
  
  public DriveSubsystem() {

FrontLeft = new WPI_TalonFX(Constants.OperatorConstants.Front_Left_DriveID);
RearLeft = new WPI_TalonFX(Constants.OperatorConstants.Rear_Left_DriveID);
FrontRight = new WPI_TalonFX(Constants.OperatorConstants.Front_Right_DriveID);
RearRight = new WPI_TalonFX(Constants.OperatorConstants.Rear_Right_DriveID);

LeftDrive = new MotorControllerGroup(FrontLeft, RearLeft);
LeftDrive.setInverted(true);

RightDrive = new MotorControllerGroup(FrontRight, RearRight);

m_Drive = new DifferentialDrive(LeftDrive, RightDrive);

  }


public void ArcadeDrive(double xspeed, double RotateSpeed){

m_Drive.arcadeDrive(xspeed, RotateSpeed);

}

public double getleftencoder(){

  return FrontLeft.getSelectedSensorPosition();
}


public void TankDrive(double LeftSpeed, double RightSpeed){

  m_Drive.tankDrive(LeftSpeed, RightSpeed);
}


public double getRightEncoder(){

 return  FrontRight.getSelectedSensorPosition(); //////////////////////////////////////////////////////////

}










  @Override
  public void periodic() {


    SmartDashboard.putNumber("Left Encoder", FrontLeft.getSelectedSensorPosition());

    SmartDashboard.putNumber("Right Encoder", FrontRight.getSelectedSensorPosition());

    // This method will be called once per scheduler run
  }
}
