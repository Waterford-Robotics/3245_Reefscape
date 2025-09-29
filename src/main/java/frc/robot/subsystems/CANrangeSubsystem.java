package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANrangeSubsystem extends SubsystemBase{
  
  private CANrange m_canrange;

  public CANrangeSubsystem() {
    m_canrange = new CANrange(41);
  }

  public boolean getIsDetected() {
    return m_canrange.getDistance().getValue().isNear(Inches.of(2), 0.1);
}

  public void periodic() {}
}
