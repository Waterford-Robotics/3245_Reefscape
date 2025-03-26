package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;

public class AlgaeSubsystem extends SubsystemBase{

  private TalonFX m_algaeArm;
  private TalonFX m_algaeIntake;

  private TalonFXConfiguration algaeArmConfig;
  private TalonFXConfiguration algaeIntakeConfig;

  private Angle lastDesiredPosition;

  public AlgaeSubsystem() {
    
    m_algaeArm = new TalonFX(MotorIDConstants.k_algaeArmID);
    m_algaeIntake = new TalonFX(MotorIDConstants.k_algaeIntakeID);

    lastDesiredPosition = Units.Rotations.of(0);

    algaeArmConfig = new TalonFXConfiguration();
    algaeArmConfig.Slot0.kP = MotorPIDConstants.k_algaeArmkP;
    algaeArmConfig.Slot0.kI = MotorPIDConstants.k_algaeArmkI;
    algaeArmConfig.Slot0.kD = MotorPIDConstants.k_algaeArmkD;
    algaeArmConfig.Slot0.kS = MotorPIDConstants.k_algaeArmkS;
    algaeArmConfig.Slot0.kV = MotorPIDConstants.k_algaeArmkV;
    algaeArmConfig.Slot0.kG = MotorPIDConstants.k_algaeArmkG;

    algaeArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeArmConfig.CurrentLimits.SupplyCurrentLimit = AlgaeConstants.k_algaeSupplyCurrentLimit;
    algaeArmConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
    algaeArmConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(0.7).in(Units.Rotations); 
    algaeArmConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    algaeArmConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(0).in(Units.Rotations); // Starting position
    algaeArmConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    algaeArmConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // algaeArmConfig.Feedback.SensorToMechanismRatio = 0.4545;

    algaeIntakeConfig = new TalonFXConfiguration();

    // Kraken Configs
    algaeIntakeConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = AlgaeConstants.k_algaeRampRate;
    algaeIntakeConfig.MotorOutput.PeakForwardDutyCycle = AlgaeConstants.k_algaeClosedMaxSpeed;
    algaeIntakeConfig.MotorOutput.PeakReverseDutyCycle = -AlgaeConstants.k_algaeClosedMaxSpeed;
    algaeIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeIntakeConfig.CurrentLimits.SupplyCurrentLimit = AlgaeConstants.k_algaeSupplyCurrentLimit;

    m_algaeArm.getConfigurator().apply(algaeArmConfig, 0.05);
    m_algaeIntake.getConfigurator().apply(algaeIntakeConfig, 0.05);
  }

    public Angle getWristPosition(){
      return Units.Rotations.of(m_algaeArm.get());
    }

    public void setPosition(Angle angle){
      m_algaeArm.setControl(new PositionVoltage(angle.in(Units.Rotations)));
      lastDesiredPosition = angle;
    }

    public void setNeutral() {
      m_algaeArm.setControl(new NeutralOut());
    }

    public void resetSensorPosition(Angle setpoint) {
      m_algaeArm.setPosition(setpoint.in(Units.Rotations));
    }

    public void shoot(){
      m_algaeIntake.set(0.1);
    }

    public void stopShooter(){
      m_algaeIntake.set(0);
    }

    public void intake(){
      m_algaeIntake.set(-0.5);
    }

    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Algae/Pos", m_algaeArm.getPosition().getValueAsDouble());

      /*
      SmartDashboard.putString("Algae/Units", m_algaeArm.getPosition().getUnits());
      SmartDashboard.putNumber("Algae/CLO", m_algaeArm.getClosedLoopOutput().getValueAsDouble());
      SmartDashboard.putNumber("Algae/Output", m_algaeArm.get());
      SmartDashboard.putNumber("Algae/Inverted", m_algaeArm.getAppliedRotorPolarity().getValueAsDouble());
      SmartDashboard.putNumber("Algae/Current", m_algaeArm.getSupplyCurrent().getValueAsDouble());
      */
  }
}
