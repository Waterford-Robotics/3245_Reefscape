package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.SensorIDConstants;

public class CANRangeSubsystem extends SubsystemBase{
  
  private CANrange m_canrange;

  public CANRangeSubsystem() {
    m_canrange = new CANrange(SensorIDConstants.k_intakeCANRangeID, "Elevator/Coral");
    m_canrange.getConfigurator().apply(IntakeConfigs.INTAKE_CANRANGE_CONFIGURATION, 0.05);
  }

  public boolean getIsDetected() {
    return m_canrange.getDistance().getValue().isNear(Inches.of(2.5), 0.9);
}

  public void periodic() {
    SmartDashboard.putBoolean("canrange", getIsDetected());
  }
}