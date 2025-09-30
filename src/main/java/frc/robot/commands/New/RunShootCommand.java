package frc.robot.commands.New;

import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

// Runs Shot
public class RunShootCommand extends Command {

  // Intake and CANRange
  IntakeSubsystem m_intakeSubsystem;
  CANRangeSubsystem m_CANrangesubsystem;
  boolean m_positioned;

  // Constructor
  public RunShootCommand(IntakeSubsystem intakeSubsystem, CANRangeSubsystem CANrangesubsystem, boolean positioned) {
        
    // Definitions and setting parameters are equal to members!
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    m_CANrangesubsystem = CANrangesubsystem;
    addRequirements(CANrangesubsystem);

    // Position Status
    m_positioned = positioned;
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {
    if(m_CANrangesubsystem.getIsDetected()) {
      m_intakeSubsystem.shoot();
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {}

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return !m_CANrangesubsystem.getIsDetected();
  }
}