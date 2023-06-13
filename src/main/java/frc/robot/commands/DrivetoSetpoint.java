// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DrivetoSetpoint extends CommandBase {

private PIDController m_Left_Drive_Controller;

private PIDController m_Right_Drive_Controller;

private final DriveSubsystem m_DriveSubsystem;

private double setpoint;




  /** Creates a new DrivetoSetpoint. */
  public DrivetoSetpoint(DriveSubsystem m_DriveSubsystem, double m_setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
this.m_DriveSubsystem = m_DriveSubsystem;

this.setpoint =setpoint; 

addRequirements(m_DriveSubsystem);



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_Left_Drive_Controller = new PIDController(SmartDashboard.getNumber("DriveKP",Constants.DrivePID.DrivekP), SmartDashboard.getNumber("DriveKI", Constants.DrivePID.DrivekI), Constants.DrivePID.DrivekD);
    m_Left_Drive_Controller.setTolerance(Constants.DrivePID.tolerance);

    m_Right_Drive_Controller = new PIDController(SmartDashboard.getNumber("DriveKP",Constants.DrivePID.DrivekP), SmartDashboard.getNumber("DriveKI", Constants.DrivePID.DrivekI), Constants.DrivePID.DrivekD);
    m_Right_Drive_Controller.setTolerance(Constants.DrivePID.tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double feedforward = Constants.DrivePID.DriveFF;
    double speed = m_Left_Drive_Controller.calculate(m_DriveSubsystem.getleftencoder(),setpoint);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    speed = (speed > 1 ) ? 1.0 : speed;
    speed = (speed < -1 ) ? -1 : speed; 


    double feedforward2 = Constants.DrivePID.DriveFF;
    double speed2 = m_Right_Drive_Controller.calculate(m_DriveSubsystem.getRightEncoder(),setpoint);
    speed2 = (speed2 > 0) ? speed2 + feedforward2 : speed2 - feedforward2;
    speed2 = (speed2 > 1 ) ? 1.0 : speed2;
    speed2 = (speed2 < -1 ) ? -1 : speed2; 









    m_DriveSubsystem.TankDrive(-speed*Constants.OperatorConstants.Max_Drive_Speed,-speed2*Constants.OperatorConstants.Max_Drive_Speed);
   
    SmartDashboard.putNumber("pidspeed", speed);







  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_DriveSubsystem.TankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
