// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeAndShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.IntakeAndShooterSubsystem.ShooterMode;

import static edu.wpi.first.units.Units.*;

import org.json.simple.JSONObject;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public final SwerveDriveSubsystem m_swerveDriveSubsystem;
  public final IntakeAndShooterSubsystem m_intakeShooterSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandGenericHID m_driverController =
      new CommandGenericHID(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_secondaryController =
      new CommandXboxController(1);

  private static RobotContainer instance;

  private SendableChooser<Command> autoChooser;

  public static RobotContainer getStaticInstance() {
    return instance;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    instance = this;

    this.m_swerveDriveSubsystem = new SwerveDriveSubsystem(
      () -> this.m_driverController.getRawAxis(1), 
      () -> this.m_driverController.getRawAxis(0), 
      () -> -this.m_driverController.getRawAxis(2)
      );
    this.m_intakeShooterSubsystem = new IntakeAndShooterSubsystem();
  

    NamedCommands.registerCommand("shootCharge", new InstantCommand(() -> m_intakeShooterSubsystem.setOutput(1.0, ShooterMode.CHARGE)) );
    NamedCommands.registerCommand("shootRelease", new InstantCommand(() -> m_intakeShooterSubsystem.setOutput(0.4, ShooterMode.FIRE)) );
    NamedCommands.registerCommand("shootStop", new InstantCommand(() -> m_intakeShooterSubsystem.setOutput(ShooterMode.STOP)) );
    NamedCommands.registerCommand("shoot", new SequentialCommandGroup(
      new InstantCommand(() -> m_intakeShooterSubsystem.setOutput(1.0, ShooterMode.CHARGE)),
      new WaitCommand(0.5),
      new InstantCommand(() -> m_intakeShooterSubsystem.setOutput(0.4, ShooterMode.FIRE)),
      new WaitCommand(0.5),
      new InstantCommand(() -> m_intakeShooterSubsystem.setOutput(ShooterMode.STOP))
    ));
    NamedCommands.registerCommand("intakeStart", new InstantCommand(() -> m_intakeShooterSubsystem.setOutput(0.4, ShooterMode.FIRE)));
    NamedCommands.registerCommand("intakeStop", new InstantCommand(() -> m_intakeShooterSubsystem.setOutput(ShooterMode.STOP)));



    this.autoChooser = AutoBuilder.buildAutoChooser();

    Shuffleboard.getTab("Driver Panel").add("Intake Camera", CameraServer.startAutomaticCapture()).withSize(6, 5).withPosition(0, 0);
    Shuffleboard.getTab("Driver Panel").add("Auto Chooser", this.autoChooser).withSize(2, 1).withPosition(6, 0);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    m_driverController.button(3).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.resetHeading()));
    // m_driverController.button(4).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.setTrajectory("Test")));
    // m_driverController.button(6).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.cancelMove()));
    // m_driverController.button(5).onTrue(m_intakeShooterSubsystem.hold());

    Command secondControllerCommand = new Command() {

      @Override
      public void execute() {
        if (DriverStation.isAutonomousEnabled()) return;
        if (m_secondaryController.getLeftTriggerAxis() > 0.1) {
          m_intakeShooterSubsystem.setOutput(m_secondaryController.getHID().getXButton() ? -0.4 : 0.4, ShooterMode.FIRE);
        } else if (m_secondaryController.getRightTriggerAxis() > 0.05) {
          m_intakeShooterSubsystem.setOutput(m_secondaryController.getHID().getXButton() ? -m_secondaryController.getRightTriggerAxis() : m_secondaryController.getRightTriggerAxis(), ShooterMode.CHARGE);
        } else {
          m_intakeShooterSubsystem.setOutput(ShooterMode.STOP);
          m_intakeShooterSubsystem.setIntake(m_driverController.getHID().getRawButton(1));
        }
      }
    };
    
    secondControllerCommand.addRequirements(m_intakeShooterSubsystem);
    secondControllerCommand.setName("Teleop Intake");

    m_intakeShooterSubsystem.setDefaultCommand(secondControllerCommand);

    m_driverController.button(7).whileTrue(new Command() {
      private boolean on = true;

      @Override
      public void initialize() {
        on = !on;
        m_swerveDriveSubsystem.setFieldCentric(on);
      }
    });

    m_driverController.povUp().whileTrue(new RunCommand(() -> m_intakeShooterSubsystem.setElevatorTargetRaw(m_intakeShooterSubsystem.getElevatorTargetRaw() + 0.025)));
    m_driverController.povDown().whileTrue(new RunCommand(() -> m_intakeShooterSubsystem.setElevatorTargetRaw(m_intakeShooterSubsystem.getElevatorTargetRaw() - 0.025)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    var command = autoChooser.getSelected();
    // if (command != null) {
    //   command.addRequirements(m_intakeShooterSubsystem);
    //   command.addRequirements(m_swerveDriveSubsystem);
    // }
    return command;
  }
}
