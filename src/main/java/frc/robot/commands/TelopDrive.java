// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TelopDrive extends CommandBase {
  /** Creates a new TelopDrive. */

  DoubleSupplier xspeed1;
  DoubleSupplier xRoation;

  public final DriveSubsystem m_DrivesSubsystem;

  public TelopDrive(DriveSubsystem m_DriveSubsystem, DoubleSupplier xspeed, DoubleSupplier RoationSpeed) {

    this.xRoation = RoationSpeed;
    this.xspeed1 = xspeed;
    this.m_DrivesSubsystem = m_DriveSubsystem;

    addRequirements(m_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


m_DrivesSubsystem.ArcadeDrive(xspeed1.getAsDouble()*Constants.OperatorConstants.Max_Drive_Speed , xRoation.getAsDouble()*Constants.OperatorConstants.Max_Rotation_Speeed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

m_DrivesSubsystem.ArcadeDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
