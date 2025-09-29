package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.SensorIDConstants;

public class IntakeSubsystem extends SubsystemBase{
  
  // Kraken x60
  private TalonFX m_shooter;

  // CANRange
  private CANrange m_intakeCANRange;

  public IntakeSubsystem() {
    
    m_shooter = new TalonFX(MotorIDConstants.k_shooterKrakenID, "Elevator/Coral");
    m_intakeCANRange = new CANrange(SensorIDConstants.k_intakeCANRange, "Elevator/Coral");

    m_shooter.getConfigurator().apply(IntakeConfigs.INTAKE_TALON_FX_CONFIGURATION, 0.05);
    m_intakeCANRange.getConfigurator().apply(IntakeConfigs.INTAKE_CANRANGE_CONFIGURATION, 0.05);

    MotorConstants.k_orchestra.addInstrument(m_shooter); 
  }

  // Shooter Stuff
  public void shoot() {
    m_shooter.set(WristConstants.k_shootSpeed);
  }

  public void shootSpeed(double power) {
    m_shooter.set(power);
  }

  public void stopShooter() {
    m_shooter.set(0);
  }

  public void intake() {
    m_shooter.set(-WristConstants.k_intakeSpeed);
  }

  // Range Stuff
  public boolean getIsDetected() {
    return m_intakeCANRange.getIsDetected().getValue();
  }

  public double getDistance() {
    return m_intakeCANRange.getDistance().getValueAsDouble();
  }

  public double getSignalStrength() {
    return m_intakeCANRange.getSignalStrength().getValueAsDouble();
  }

  public void periodic() {
    SmartDashboard.putBoolean("Intake/CANRange/Coral Detected", getIsDetected());
    SmartDashboard.putNumber("Intake/CANRange/Intake Distance", getDistance());
    SmartDashboard.putNumber("Intake/CANRange/Intake Signal Strength", getSignalStrength());
  }
}
