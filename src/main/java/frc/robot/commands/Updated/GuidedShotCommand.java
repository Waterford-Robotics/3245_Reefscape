// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Updated;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Shoots Pieces with CANRange Assistance
public class GuidedShotCommand extends Command {

  // Uses Elevator and Subsystems
  IntakeSubsystem m_intakeSubsystem;
  double m_speed;
  boolean m_isTimeRecorded;
  Timer m_timer = new Timer();

  // Constructor
  public GuidedShotCommand(IntakeSubsystem intakeSubsystem, double speed) {
        
    // Definitions and setting parameters are equal to members!
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);

    // Speed
    m_speed = speed;
  }

  // Reset timer when the command starts executing
  public void initialize() {
    m_isTimeRecorded = false;
  }
  
  // Actual command
  public void execute() {

    m_intakeSubsystem.shootSpeed(m_speed);

    if(!m_intakeSubsystem.getIsDetected() && !m_isTimeRecorded) {
      m_timer.start();
      m_timer.reset();
      m_isTimeRecorded = true;
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopShooter();
    VisionConstants._positioned = false;
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return !VisionConstants._positioned || m_timer.get() > WristConstants.k_extendedShotTime;
  }
}
