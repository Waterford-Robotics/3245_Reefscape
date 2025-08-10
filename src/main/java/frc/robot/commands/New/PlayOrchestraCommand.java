
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.New;

import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

// Let the people rejoice!
public class PlayOrchestraCommand extends Command {

  // Use all subsystems
  SwerveSubsystem m_swerveSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  WristSubsystem m_wristSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  // Constructor
  public PlayOrchestraCommand(
    SwerveSubsystem swerveSubsystem, 
    ElevatorSubsystem elevatorSubsystem, 
    WristSubsystem wristSubsystem, 
    IntakeSubsystem intakeSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
    m_wristSubsystem = wristSubsystem;
    addRequirements(wristSubsystem);
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Init
  public void initialize() {
  }
  
  // Actual command
  public void execute() {

    // Load and play
    MotorConstants.k_orchestra.loadMusic("Nessy Demo Track 1 - TETORIS.chrp");
    MotorConstants.k_orchestra.play();
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
