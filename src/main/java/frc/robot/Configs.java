package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.units.Units;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorPIDConstants;
import frc.robot.Constants.SensorIDConstants;
import frc.robot.Constants.WristConstants;

public final class Configs {
    
  public static final class ElevatorConfigs {

		// Elevator Kraken x60 - (QTY 2)
    public static final TalonFXConfiguration ELEVATOR_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    // Elevator WCP Cancoder
    public static final CANcoderConfiguration ELEVATOR_CANCODER_CONFIGURATION = new CANcoderConfiguration();

		static {

      /*
       **********************************************
       **    ELEVATOR KRAKEN x60 CONFIGURATIONS    **
       **********************************************
      */

			// PID Stuff
			ELEVATOR_TALON_FX_CONFIGURATION.Slot0.kP = MotorPIDConstants.k_elevatorkP;
			ELEVATOR_TALON_FX_CONFIGURATION.Slot0.kI = MotorPIDConstants.k_elevatorkI;
			ELEVATOR_TALON_FX_CONFIGURATION.Slot0.kD = MotorPIDConstants.k_elevatorkD;
			ELEVATOR_TALON_FX_CONFIGURATION.Slot0.kS = MotorPIDConstants.k_elevatorkS;
			ELEVATOR_TALON_FX_CONFIGURATION.Slot0.kV = MotorPIDConstants.k_elevatorkV;
			ELEVATOR_TALON_FX_CONFIGURATION.Slot0.kG = MotorPIDConstants.k_elevatorkG;
			ELEVATOR_TALON_FX_CONFIGURATION.Slot0.kA = MotorPIDConstants.k_elevatorkA;

			// Kraken Configs
			ELEVATOR_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			ELEVATOR_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.k_elevatorSupplyCurrentLimit;
			ELEVATOR_TALON_FX_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

			// Motor Limitations
			ELEVATOR_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; // No breaking elevator
			ELEVATOR_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(49).in(Units.Inches);
			ELEVATOR_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
			ELEVATOR_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0).in(Units.Inches); // Starting position

			// Elevator Mode
			ELEVATOR_TALON_FX_CONFIGURATION.Slot0.GravityType = GravityTypeValue.Elevator_Static;
			
			// Fused CANCoder and stuff
      ELEVATOR_TALON_FX_CONFIGURATION.Feedback.FeedbackRemoteSensorID = SensorIDConstants.k_elevatorCANCoderID;
      ELEVATOR_TALON_FX_CONFIGURATION.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

      // Elevator motors will provide feedback in INCHES the carriage has moved
			ELEVATOR_TALON_FX_CONFIGURATION.Feedback.SensorToMechanismRatio = 0.0909; // 0.4545
      ELEVATOR_TALON_FX_CONFIGURATION.Feedback.RotorToSensorRatio = 5;


      /*
       ********************************************
       **    ELEVATOR CANCODER CONFIGURATIONS    **
       ********************************************
      */
      
      // TODO: CHECK VALUES
      ELEVATOR_CANCODER_CONFIGURATION.MagnetSensor.MagnetOffset = ElevatorConstants.k_elevatorCANCoderOffset;
      ELEVATOR_CANCODER_CONFIGURATION.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		}
  }

  public static final class WristConfigs {

    // Wrist Kraken x60
    public static final TalonFXConfiguration WRIST_TALON_FX_CONFIGURATION = new TalonFXConfiguration();

    static {

      /*
       *******************************************
       **    WRIST KRAKEN x60 CONFIGURATIONS    **
       *******************************************
      */

      WRIST_TALON_FX_CONFIGURATION.Slot0.kP = MotorPIDConstants.k_wristP;
      WRIST_TALON_FX_CONFIGURATION.Slot0.kI = MotorPIDConstants.k_wristI;
      WRIST_TALON_FX_CONFIGURATION.Slot0.kD = MotorPIDConstants.k_wristD;
      WRIST_TALON_FX_CONFIGURATION.Slot0.kS = MotorPIDConstants.k_wristS;
      WRIST_TALON_FX_CONFIGURATION.Slot0.kV = MotorPIDConstants.k_wristV;
      WRIST_TALON_FX_CONFIGURATION.Slot0.kG = MotorPIDConstants.k_wristG;

      WRIST_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      WRIST_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = WristConstants.k_supplyCurrentLimit;
      WRIST_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
      WRIST_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(5).in(Units.Rotations);
      WRIST_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      WRIST_TALON_FX_CONFIGURATION.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(0).in(Units.Rotations); // Starting position
      WRIST_TALON_FX_CONFIGURATION.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

      WRIST_TALON_FX_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
  }

  public static final class IntakeConfigs {

    // Intake Kraken x60
    public static final TalonFXConfiguration INTAKE_TALON_FX_CONFIGURATION = new TalonFXConfiguration();
    
    // Intake CANRange
    public static final CANrangeConfiguration INTAKE_CANRANGE_CONFIGURATION = new CANrangeConfiguration();
    
    static {

      /*
       ********************************************
       **    INTAKE KRAKEN x60 CONFIGURATIONS    **
       ********************************************
      */

      INTAKE_TALON_FX_CONFIGURATION.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = WristConstants.k_shooterRampRate;
      INTAKE_TALON_FX_CONFIGURATION.MotorOutput.PeakForwardDutyCycle = WristConstants.k_shooterClosedMaxSpeed;
      INTAKE_TALON_FX_CONFIGURATION.MotorOutput.PeakReverseDutyCycle = -WristConstants.k_shooterClosedMaxSpeed;
      INTAKE_TALON_FX_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      INTAKE_TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit = WristConstants.k_intakeSupplyCurrentLimit;


      /*
       ******************************************
       **    INTAKE CANRANGE CONFIGURATIONS    **
       ******************************************
      */

      // Distance stuff uh huh
      INTAKE_CANRANGE_CONFIGURATION.ProximityParams.MinSignalStrengthForValidMeasurement = 2500; // TODO: Make it bigger?
      INTAKE_CANRANGE_CONFIGURATION.ProximityParams.ProximityThreshold = 0.02; // TODO: Measuring time omg
      INTAKE_CANRANGE_CONFIGURATION.ProximityParams.ProximityHysteresis = 0.01;
      
      INTAKE_CANRANGE_CONFIGURATION.ToFParams.UpdateFrequency = 50;
      INTAKE_CANRANGE_CONFIGURATION.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;   
    }
  }
}
