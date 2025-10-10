// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ReactConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.New.PlayOrchestraCommand;
import frc.robot.commands.New.ResetGyroCommand;
import frc.robot.commands.New.RunShootCommand;
import frc.robot.commands.New.SetElevatorReactCommand;
import frc.robot.commands.New.SetLevelReactCommand;
import frc.robot.commands.New.SetSideReactCommand;
import frc.robot.commands.New.StopOrchestraCommand;
import frc.robot.commands.New.TriggerElevatorCommand;
import frc.robot.commands.Updated.AimNRangeAutoCoralStationCommand;
import frc.robot.commands.Updated.AimNRangeCommand;
import frc.robot.commands.Updated.LEDColorChangeCommand;
import frc.robot.commands.Updated.NeutralElevatorCommand;
import frc.robot.commands.Updated.RunIntakeForSecsAutoCommand;
import frc.robot.commands.Updated.RunIntakeForSecsCommand;
import frc.robot.commands.Updated.RunShootForSecsSpeedCommand;
import frc.robot.commands.Updated.SetElevatorCommand;
import frc.robot.commands.Updated.SetWristCommand;
import frc.robot.commands.Updated.ZeroElevatorCommand;
import frc.robot.commands.Updated.ZeroWristCommand;
import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.Dashboard.AutoDashboardSubsystem;
import frc.robot.subsystems.Dashboard.ReactDashSubsystem;
import frc.robot.subsystems.Dashboard.TeleopDashboardSubsystem;
import frc.robot.subsystems.Limelight.LimelightSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final CANRangeSubsystem m_canRangeSubsystem = new CANRangeSubsystem();

  @SuppressWarnings("unused")
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

  // REACT
  @SuppressWarnings("unused")
  private final ReactDashSubsystem m_reactDashSubsystem = new ReactDashSubsystem();

  @SuppressWarnings("unused")
  private final AutoDashboardSubsystem m_autoDashboardSubsystem = new AutoDashboardSubsystem();

  @SuppressWarnings("unused")
  private final TeleopDashboardSubsystem m_teleopDashboardSubsystem = new TeleopDashboardSubsystem();

  // Create New Choosing Option in SmartDashboard for Autos
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.k_driverControllerPort);
  // private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.k_operatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // Makes the drive command the default command (good!)
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // Named Command Configuration
    NamedCommands.registerCommand("Raise Wrist", RaiseWristCommand());
    NamedCommands.registerCommand("Intake", IntakeAutoCommand());
    NamedCommands.registerCommand("Score L4 Left", AimNRangeScoreAutoLeftCommand());
    NamedCommands.registerCommand("Score L4 Right", AimNRangeScoreAutoRightCommand());
    NamedCommands.registerCommand("Zero Gyro", new InstantCommand(() -> m_swerveSubsystem.zeroGyro(), m_swerveSubsystem));
    NamedCommands.registerCommand("Position Coral Station Left", positionNIntakeAutoCoralStationLeft());
    NamedCommands.registerCommand("Position Coral Station Right", positionNIntakeAutoCoralStationRight());

    // COMP AUTOS
    // m_chooser.addOption("3C: BL-R, FL-R, FL-L REVISED", m_swerveSubsystem.getAutonomousCommand("3C BL-R, FL-R, FL-L REVISED")); // U (P)
    // m_chooser.addOption("3C BR-L, FR-L, FR-R *", m_swerveSubsystem.getAutonomousCommand("3C BR-L, FR-L, FR-R")); // U (P)

    // Puts a chooser on the SmartDashboard!
    SmartDashboard.putData("AutoMode", m_chooser);
  }

  // Trigger & Button Bindings!
  private void configureBindings() {
    
    /*
     * DRIVER CONTROLLER
     * Right Bump - Raise Wrist
     * Left Bump - Lower Wrist
     * Right Trig - Intake and Set X
     * Start - Reset Gyro
     * Back - Stop Orchestra
     */
    
    // Raise Wrist - Right Bump
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_rightbump)
      .onTrue(
        RaiseWristCommand()
      );

    // Lower Wrist - Left Bump
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_leftbump)
      .onTrue(
        LowerWristCommand()
      );

    // Score Coral - A
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_A)
      .onTrue(
        AimNRangeScoreCommand()
          .onlyIf(() -> (m_canRangeSubsystem.getIsDetected() && ReactConstants._tiv)
        )
      );

    // Intake and Set X - Right Trig
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem)
          .until(() -> m_canRangeSubsystem.getIsDetected()
        )
      )
      .onFalse(
        new InstantCommand(() -> m_intakeSubsystem.stopShooter(), m_intakeSubsystem)
      );

    // Reset Gyro - Start Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_start)
      .onTrue(
        new InstantCommand(() -> m_swerveSubsystem.zeroGyro(), m_swerveSubsystem)
      );
    
    // Stop Orchestra - Back Button
    new JoystickButton(m_driverController.getHID(), ControllerConstants.k_back)
    .onTrue(
      new StopOrchestraCommand(m_swerveSubsystem, m_elevatorSubsystem, m_wristSubsystem, m_intakeSubsystem)
    );

    /*
     * TRIGGERS
     * Reset Gyro
     * Elevator Raise to Preset
     * Elevator Raise Manual
     * Reset Elevator - Automatic
     * Neutral Elevator - Manual
     * Zero Elevator - Manual
     * Score Coral
     * LEDs - Five Statuses
     * Play Orchestra
     */

    // REACT DASH STUFF!!
    new Trigger(() -> ReactConstants._resetGyro)
      .onTrue(new ResetGyroCommand(m_swerveSubsystem)
    );

    new Trigger(() -> ReactConstants._triggerElevatorScore)
      .onTrue(new SetElevatorReactCommand(m_elevatorSubsystem)
    );

    new Trigger(() -> ReactConstants._raiseElevator)
      .onTrue(new SetElevatorCommand(m_elevatorSubsystem, "L3"))
      .onFalse(ResetElevatorCommand()
    );

    new Trigger(() -> ReactConstants._triggerElevatorReset)
      .onTrue(ResetElevatorCommand()
    );

    new Trigger(() -> ReactConstants._neutralElevator)
      .onTrue(new NeutralElevatorCommand(m_elevatorSubsystem)
    );

    new Trigger(() -> ReactConstants._zeroElevator)
      .onTrue(new ZeroElevatorCommand(m_elevatorSubsystem)
    );

    new Trigger(() -> ReactConstants._scoreCoral)
      .onTrue(AimNRangeScoreCommand()
    );

    new Trigger(() -> ReactConstants._triggerLEDScoreSignal)
      .onTrue(new InstantCommand(() -> m_ledSubsystem.setRainbowRainbow(), m_ledSubsystem)
    );

    new Trigger(() -> ReactConstants._triggerLEDManualSignal)
      .onTrue(new InstantCommand(() -> m_ledSubsystem.setRainbowParty(), m_ledSubsystem)
    );

    new Trigger(() -> ReactConstants._triggerLEDParkSignal)
      .onTrue(new InstantCommand(() -> m_ledSubsystem.setStrobeWhite(), m_ledSubsystem)
    );

    new Trigger(() -> ReactConstants._triggerLEDAllianceSignal)
      .onTrue(new InstantCommand(() -> m_ledSubsystem.setAllianceColor(), m_ledSubsystem)
    );

    new Trigger(() -> ReactConstants._triggerLEDLimelightSignal)
      .onTrue(new InstantCommand(() -> m_ledSubsystem.setColorWavesForestLimelight(), m_ledSubsystem)
    );

    new Trigger(() -> ReactConstants._tivCoral)
      .onTrue(new InstantCommand(() -> m_ledSubsystem.setStrobeWhite(), m_ledSubsystem)
    );

    new Trigger(() -> ReactConstants._playOrchestra)
      .onTrue(new PlayOrchestraCommand(m_swerveSubsystem, m_elevatorSubsystem, m_wristSubsystem, m_intakeSubsystem)
    );
  }
  
  // Commands!
  // Command that takes Xbox Controller Inputs and allows robot to drive
  // NOTE: getLeftY and getLeftX are opposite for a reason!!! It is correct!!
  public Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> m_driverController.getRightX() * DriveConstants.k_turnRate);

  // Use this to pass the autonomous command to Robot.java
  // Returns the command to run in autonomous
  public Command getAutonomousCommand() {

    // The selected auto on SmartDashboard will be run in autonomous
    // return m_chooser.getSelected();

    // The selected auto will be run in autonomous (or not haha)
    if (ReactConstants._selectedAuto.equals("NONE")) return null;
    else return m_swerveSubsystem.getAutonomousCommand(ReactConstants._selectedAuto);
    // return m_swerveSubsystem.getAutonomousCommand("3C-BLR-FLL-FLR");
  }

  // Command Chain for Raising Wrist
  public SequentialCommandGroup RaiseWristCommand() {
    return new SequentialCommandGroup(
      new ZeroWristCommand(m_wristSubsystem),
      new SetWristCommand(m_wristSubsystem, "INTAKE")
    );
  }

  // Command Chain for Lowering Wrist
  public SequentialCommandGroup LowerWristCommand() {
    return new SequentialCommandGroup(
      new SetWristCommand(m_wristSubsystem, "SCORE"),
      new ZeroWristCommand(m_wristSubsystem)
    );
  }

  // Command Chain for Manual Reset
  public SequentialCommandGroup ResetElevatorCommand() {
    return new SequentialCommandGroup(
      new SetElevatorCommand(m_elevatorSubsystem, "zero"),
      new ZeroElevatorCommand(m_elevatorSubsystem) 
    );
  }

  // Command Chain for Intake Auto
  public SequentialCommandGroup IntakeAutoCommand() {
    return new SequentialCommandGroup(
      new SetWristCommand(m_wristSubsystem, "INTAKE"), 
      new RunIntakeForSecsCommand(m_intakeSubsystem, 1.0)
    );
  }

  // Command Chain for Positioning and Loading at the Left Side of the Coral Station in Auto
  public ParallelDeadlineGroup positionNIntakeAutoCoralStationLeft() {
    return new ParallelDeadlineGroup(
      new AimNRangeAutoCoralStationCommand(m_swerveSubsystem, true),
      new RunIntakeForSecsAutoCommand(m_intakeSubsystem, m_canRangeSubsystem, 3)
    );
  }

  // Command Chain for Positioning and Loading at the Right Side of the Coral Station in Auto
  public ParallelDeadlineGroup positionNIntakeAutoCoralStationRight() {
    return new ParallelDeadlineGroup(
      new AimNRangeAutoCoralStationCommand(m_swerveSubsystem, true),
      new RunIntakeForSecsAutoCommand(m_intakeSubsystem, m_canRangeSubsystem, 3)
    );
  }

  // Command Chain for Completely Automated Scoring
  public SequentialCommandGroup AimNRangeScoreCommand() {
    return new SequentialCommandGroup(
      new TriggerElevatorCommand("SCORE"),
      new LEDColorChangeCommand("SCORE_SIGNAL"),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new AimNRangeCommand(m_swerveSubsystem),
          new SequentialCommandGroup(
            new SetWristCommand(m_wristSubsystem, "SCORE"),
            new ZeroWristCommand(m_wristSubsystem)
          )
        ),
        new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
      ),
      new RunShootCommand(m_intakeSubsystem, m_canRangeSubsystem, VisionConstants._positioned),
      new RunShootForSecsSpeedCommand(m_intakeSubsystem, WristConstants.k_extendedShotTime, VisionConstants._positioned, 0.5),
      new ZeroWristCommand(m_wristSubsystem),
      new SetWristCommand(m_wristSubsystem, "INTAKE"),
      new TriggerElevatorCommand("RESET"),
      new LEDColorChangeCommand("NONE")
    );
  }

  // Automated Scoring in Autonomous Period - Right
  public SequentialCommandGroup AimNRangeScoreAutoRightCommand() {
    return new SequentialCommandGroup(
      new SetLevelReactCommand("4"),
      new SetSideReactCommand("R"),
      new TriggerElevatorCommand("SCORE"),
      new LEDColorChangeCommand("SCORE_SIGNAL"),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new AimNRangeCommand(m_swerveSubsystem),
          new SequentialCommandGroup(
            new SetWristCommand(m_wristSubsystem, "SCORE"),
            new ZeroWristCommand(m_wristSubsystem)
          )
        ),
        new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
      ),
      new RunShootCommand(m_intakeSubsystem, m_canRangeSubsystem, VisionConstants._positioned),
      new RunShootForSecsSpeedCommand(m_intakeSubsystem, WristConstants.k_extendedShotTime, VisionConstants._positioned, 0.5),
      new ZeroWristCommand(m_wristSubsystem),
      new SetWristCommand(m_wristSubsystem, "INTAKE"),
      new TriggerElevatorCommand("RESET"),
      new LEDColorChangeCommand("NONE")
    );
  }

  // Automated Scoring in Autonomous Period - Left
  public SequentialCommandGroup AimNRangeScoreAutoLeftCommand() {
    return new SequentialCommandGroup(
      new SetLevelReactCommand("4"),
      new SetSideReactCommand("L"),
      new TriggerElevatorCommand("SCORE"),
      new LEDColorChangeCommand("SCORE_SIGNAL"),
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new AimNRangeCommand(m_swerveSubsystem),
          new SequentialCommandGroup(
            new SetWristCommand(m_wristSubsystem, "SCORE"),
            new ZeroWristCommand(m_wristSubsystem)
          )
        ),
        new RunIntakeForSecsCommand(m_intakeSubsystem, 3.0)
      ),
      new RunShootCommand(m_intakeSubsystem, m_canRangeSubsystem, VisionConstants._positioned),
      new RunShootForSecsSpeedCommand(m_intakeSubsystem, WristConstants.k_extendedShotTime, VisionConstants._positioned, 0.5),
      new ZeroWristCommand(m_wristSubsystem),
      new SetWristCommand(m_wristSubsystem, "INTAKE"),
      new TriggerElevatorCommand("RESET"),
      new LEDColorChangeCommand("NONE")
    );
  }
}
