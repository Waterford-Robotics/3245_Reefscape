package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;

public class IntakeSubsystem extends SubsystemBase{
  
  // Kraken x60
  private TalonFX m_shooter;

  public IntakeSubsystem() {
    
    m_shooter = new TalonFX(MotorIDConstants.k_shooterKrakenID, "Elevator/Coral");

    m_shooter.getConfigurator().apply(IntakeConfigs.INTAKE_TALON_FX_CONFIGURATION, 0.05);

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

  public void periodic() {
  }
}
