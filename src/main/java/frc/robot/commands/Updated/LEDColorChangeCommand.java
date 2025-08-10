package frc.robot.commands.Updated;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReactConstants;

// Set LED Statuses
public class LEDColorChangeCommand extends Command {
        
  // Instantiate Stuff
  String m_color;

  // Constructor
  public LEDColorChangeCommand(String color) {

    m_color = color;
  }

  // Init
  public void initialize() {}
  
  // Run
  public void execute() {
    ReactConstants._ledSelection = m_color;
  }

  // What happens before the command finishes
  public void end(boolean interrupted) {}

  // Tells when the command finishes
  public boolean isFinished() {

    // Finish immediately
    return true;
  }
}
    
