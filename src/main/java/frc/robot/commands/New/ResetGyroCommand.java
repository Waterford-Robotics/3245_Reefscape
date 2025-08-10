// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.New;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

// Reset Gyro
public class ResetGyroCommand extends Command {

  // Uses Swerve Subsystem
  SwerveSubsystem m_swerveSubsystem;

  // Constructor
  public ResetGyroCommand(SwerveSubsystem swerveSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  // Init
  public void initialize() {
  }
  
  // Actual command
  public void execute() {
    m_swerveSubsystem.zeroGyro();
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
