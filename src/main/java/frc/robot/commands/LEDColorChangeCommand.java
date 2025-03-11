package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDColorChangeCommand extends Command {
        
    // Instantiate Stuff
    LEDSubsystem m_LEDSubsystem;

    public LEDColorChangeCommand(LEDSubsystem ledSubsystem, String color) {
        
        // Definitions and setting parameters are equal to members!
        m_LEDSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        // m_LEDSubsystem.setConfetti();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
    
