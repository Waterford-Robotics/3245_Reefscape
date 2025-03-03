package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.MotorIDConstants;

public class IntakeSubsystem extends SubsystemBase{

  private TalonFX m_shooter;

  private TalonFXConfiguration shooterConfig;

  public IntakeSubsystem() {
    
    m_shooter = new TalonFX(MotorIDConstants.k_shooterKrakenID, "Elevator/Coral");

    shooterConfig = new TalonFXConfiguration();

    // Kraken Configs
    shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = WristConstants.k_shooterRampRate;
    shooterConfig.MotorOutput.PeakForwardDutyCycle = WristConstants.k_shooterClosedMaxSpeed;
    shooterConfig.MotorOutput.PeakReverseDutyCycle = -WristConstants.k_shooterClosedMaxSpeed;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.k_supplyCurrentLimit;

    m_shooter.getConfigurator().apply(shooterConfig, 0.05);
  }

  public void shoot(){
    m_shooter.set(WristConstants.k_shootSpeed);
  }

  public void stopShooter(){
    m_shooter.set(0);
  }

  public void intake(){
    m_shooter.set(-WristConstants.k_intakeSpeed);
  }

  public void periodic() {
  }
}
