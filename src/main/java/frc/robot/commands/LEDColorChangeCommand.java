package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDColorChangeCommand extends Command {
        
  // Instantiate Stuff
  LEDSubsystem m_LEDSubsystem;
  String m_color;

  public LEDColorChangeCommand(LEDSubsystem ledSubsystem, String color) {
      
    // Definitions and setting parameters are equal to members!
    m_LEDSubsystem = ledSubsystem;
    addRequirements(ledSubsystem);

    m_color = color;
  }

  public void initialize() {}
  
  public void execute() {

    if (m_color.equals("Alliance")) {
      LEDConstants.k_allowTIV = true;
    }

    if (m_color.equals("Scoring")) {
      m_LEDSubsystem.setRainbowParty();
      LEDConstants.k_allowTIV = false;
    }
  }

  public void end(boolean interrupted) {}

  public boolean isFinished() {
      return true;
  }
}
    
