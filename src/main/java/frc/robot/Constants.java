// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public final class Constants {

  public static final class ControllerConstants {
    public static final int k_driverControllerPort = 0; // Driver
    public static final int k_operatorControllerPort = 1; // Operator

    // public static final double k_driveDeadband = 0.10; // Increase to combat larger stick drift

    public final static int k_start = Button.kStart.value; // Start Button
    public final static int k_back = Button.kBack.value; // Back Button

    public static final int k_A = Button.kA.value; // A
    public static final int k_B = Button.kB.value; // B
    public static final int k_X = Button.kX.value; // X
    public static final int k_Y = Button.kY.value; // Y

    public static final int k_dpadup = 0; // D-Pad Up
    public static final int k_dpadRight = 90; // D-Pad Right
    public static final int k_dpadDown = 180; // D-Pad Down
    public static final int k_dpadLeft = 270; // D-Pad Left

    public final static int k_rightbump = Button.kRightBumper.value; // Right Bump
    public final static int k_leftbump = Button.kLeftBumper.value; // Left Bump

    public final static int k_righttrig = Axis.kRightTrigger.value; // Right Trig
    public final static int k_lefttrig = Axis.kLeftTrigger.value; // Left Trig
  }

  // Constants for Kraken Drivetrain!
  public static final class SwerveConstants {

    // Must be max physically possible speed
    public static final double k_maxSpeed = edu.wpi.first.math.util.Units.feetToMeters(18.9); // Meters per second
    public static final double k_maxAngularSpeed = 1.5 * Math.PI; // Radians per second
  }

  // Constants for controller input!
  public static final class DriveConstants {

    // YAGSL Swerve Stuff
    public static final double k_driveDeadBand = 0.1;
    public static final double k_driveSpeed = -0.8;
    public static final double k_turnRate = -0.85;
  }

  // Constants for Motor IDs
  public static final class MotorIDConstants {

    // Elevator
    public static final int k_elevatorKrakenLeftID = 21;
    public static final int k_elevatorKrakenRightID = 22;

    // Wrist
    public static final int k_wristKrakenID = 31;
    public static final int k_shooterKrakenID = 32;

    // Algae
    public static final int k_algaeWristID = 41;
    public static final int k_algaeIntakeID = 40;
  }

  // Constants for Sensor IDs
  public static final class SensorIDConstants {

  }

  // Constants for Elevator
  public static final class ElevatorConstants {
    public static final double k_elevatorRampRate = 0.25;
    public static final double k_elevatorClosedMaxSpeed = 0.8;
    public static final int k_elevatorSupplyCurrentLimit = 40;

    public static final Distance k_zeroHeight = Units.Inches.of(0);
    public static final Distance k_coralL1Height = Units.Inches.of(0); 
    public static final Distance k_coralL2Height = Units.Inches.of(7);
    public static final Distance k_coralL3Height = Units.Inches.of(21);
    public static final Distance k_coralL4Height = Units.Inches.of(48.6);

    public static final Distance k_algaeHeight = Units.Inches.of(25);
  }

  // Constants for Wrist
  public static final class WristConstants {
    public static final double k_shooterRampRate = 0.05;
    public static final double k_shooterClosedMaxSpeed = 0.4;
    public static final int k_supplyCurrentLimit = 40;

    public static final double k_shootSpeed = 0.2;
    public static final double k_intakeSpeed = 0.15;

    public static final Angle k_wristScoreAngle = Units.Rotations.of(0);
    public static final Angle k_wristIntakeAngle = Units.Rotations.of(2.5);
  }

  // Constants for Algae
  public static final class AlgaeConstants {
    public static final double k_algaeRampRate = 0.25;
    public static final double k_algaeClosedMaxSpeed = 0.8;
    public static final int k_algaeSupplyCurrentLimit = 40;
  }

  // Constants for Motors
  public static final class MotorConstants {}

  // Constants for PID
  public static final class MotorPIDConstants {
    public static final double k_elevatorkP = 0.5;
    public static final double k_elevatorkI = 0.1;
    public static final double k_elevatorkD = 0.15;
    public static final double k_elevatorkS = 0.4;
    public static final double k_elevatorkV = 0.001;
    public static final double k_elevatorkA = 0.0;
    public static final double k_elevatorkG = 0.3;

    public static final double k_wristP = 0.9; 
    public static final double k_wristI = 0.8;
    public static final double k_wristD = 0.0;
    public static final double k_wristS = 0.4;
    public static final double k_wristV = 0.001;
    public static final double k_wristG = 0.5;

    public static final double k_algaeWristkP = 0.0;
    public static final double k_algaeWristkI = 0.0;
    public static final double k_algaeWristkD = 0.0;
    public static final double k_algaeWristkS = 0.0;
    public static final double k_algaeWristkV = 0.0;
    public static final double k_algaeWristkA = 0.0;
    public static final double k_algaeWristkG = 0.0;
  }

  // Why did the duck cross the road?
  // He wanted a quacker

  // Constants for Autonomous
  public static final class AutoConstants {}

  public static final class VisionConstants {

    // Name
    public static final String k_limelightName = "limelight-four";
    
    // Boolean for Left/Right Reef
    public static boolean k_isRightReef = true;

    // Boolean for Committing to Shoot
    public static boolean k_positioned = true;

    // Boolean for Algae Elevator Raise
    public static boolean k_isAlgaeL3 = false;

    // PID for Tag Relative Control for Scoring
    public static final double kP_aim = 0.10;
    public static final double kI_aim = 0.000;
    public static final double kD_aim = 0.000;

    public static final double kP_range = 0.35;
    public static final double kI_range = 0.0;
    public static final double kD_range = 0.0;

    public static final double kP_strafe = 0.35;
    public static final double kI_strafe = 0.0;
    public static final double kD_strafe = 0.0;

    // PID for Tag Relative Control for Algae Removal
    public static final double kP_aimAlgae = 0.04;
    public static final double kI_aimAlgae = 0.000;
    public static final double kD_aimAlgae = 0.000;

    public static final double kP_rangeAlgae = 0.35;
    public static final double kI_rangeAlgae = 0.0;
    public static final double kD_rangeAlgae = 0.0;

    public static final double kP_strafeAlgae = 0.35;
    public static final double kI_strafeAlgae = 0.0;
    public static final double kD_strafeAlgae = 0.0;

    // AimNRange Reef Right
    public static final double k_aimReefRightTarget = 0;
    public static final double k_rangeReefRightTarget = -0.54;
    public static final double k_strafeReefRightTarget = 0.16;

    // AimNRange Reef Left
    public static final double k_aimReefLeftTarget = 0;
    public static final double k_rangeReefLeftTarget = -0.54;
    public static final double k_strafeReefLeftTarget = -0.18;

    // AimNRange Algae Removal Positioning 
    public static final double k_aimAlgaePositionTarget = 0;
    public static final double k_rangeAlgaePositionTarget = -0.46;
    public static final double k_strafeAlgaePositionTarget = 0;

    // AimNRange Algae Removal Removing 
    public static final double k_aimAlgaeRemoveTarget = 0;
    public static final double k_rangeAlgaeRemoveTarget = -0.96;
    public static final double k_strafeAlgaeRemoveTarget = 0;

    // Prerequisites
    public static final double k_tzValidRange = -1.5;
    public static final double k_yawValidRange = 35;

    // Thresholds
    public static final double k_rangeThreshold = 0.03;
    public static final double k_strafeThreshold = 0.03;
    public static final double k_aimThreshold = 0.5;

    // Tag Reject Distance
    public static final int k_rejectionDistance = 3;

    // Tag Reject Rotation Rate
    public static final int k_rejectionRotationRate = 720;

    // For testing
    public static boolean k_positioning = false;
  }

  public static final class LEDConstants {
    public static final int blinkinPort = 0;
  }
}
