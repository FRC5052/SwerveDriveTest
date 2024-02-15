// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeAndShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.IntakeAndShooterSubsystem.IntakeBallCommand;
import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  private static RobotContainer instance;

  public static RobotContainer getStaticInstance() {
    return instance;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    instance = this;

    this.m_swerveDriveSubsystem = new SwerveDriveSubsystem(
      () -> -this.m_driverController.getRawAxis(0), 
      () -> this.m_driverController.getRawAxis(1), 
      () -> this.m_driverController.getRawAxis(2)
      );
    this.m_intakeShooterSubsystem = new IntakeAndShooterSubsystem();

   

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
    m_driverController.button(4).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.moveBy(new Translation2d(0, 1))));
    m_driverController.button(6).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.cancelMove()));
    m_driverController.button(2).onTrue(m_intakeShooterSubsystem.getIntakeBallCommand());
    m_driverController.button(1).whileTrue(new Command() {
      private long ticks;

      @Override
      public void initialize() {
        ticks = 0;
      }

      public double getTime() {
        return (double)ticks * Robot.kDefaultPeriod; 
      }

      @Override
      public void execute() {
          m_intakeShooterSubsystem.setShooterSpeed(((3.0/2.0) - (3.0 / (2.0 * (getTime() + 1.0)))) * ((1.0 - m_driverController.getRawAxis(3)) / 2));
          ticks++;
      }

      @Override
      public boolean isFinished() {
          return getTime() >= 1.5;
      }
    });
    m_driverController.button(1).onFalse(m_intakeShooterSubsystem.getShootBallCommand());

    m_driverController.button(8).whileTrue(new Command() {
      private boolean on = true;

      @Override
      public void initialize() {
        on = !on;
        System.out.println(on ? "Full speed enabled." : "Full speed disabled.");
        m_swerveDriveSubsystem.setFullSpeed(on);
      }
    });

    m_driverController.button(7).whileTrue(new Command() {
      private boolean on = true;

      @Override
      public void initialize() {
        on = !on;
        m_swerveDriveSubsystem.setFieldCentric(on);
      }
    });



    m_driverController.povRight().whileTrue(new RunCommand(() -> m_intakeShooterSubsystem.yawBy(Rotation2d.fromDegrees(1.0))));
    m_driverController.povLeft().whileTrue(new RunCommand(() -> m_intakeShooterSubsystem.yawBy(Rotation2d.fromDegrees(-1.0))));
    m_driverController.povUp().whileTrue(new RunCommand(() -> m_intakeShooterSubsystem.pitchBy(Rotation2d.fromDegrees(0.5))));
    m_driverController.povDown().whileTrue(new RunCommand(() -> m_intakeShooterSubsystem.pitchBy(Rotation2d.fromDegrees(-0.5))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
