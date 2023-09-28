// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveEncoder;
import frc.robot.swerve.SwerveIMU;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveMotor;

public class SwerveDriveSubsystem extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private XboxController controller;
  /** Creates a new ExampleSubsystem. */
  public SwerveDriveSubsystem() {
    PIDController pivotController = new PIDController(0.15, 0, 0.0);
    PIDController driveController = new PIDController(0.001, 0, 0);
    this.swerveDrive = new SwerveDrive(
      new Pose2d(), 
      new SwerveIMU.NavXSwerveIMU(), 
      new SwerveModule(new SwerveModule.SwerveModuleConfig() // Front Right
        .driveGearRatio(6.75)
        .pivotGearRatio(12.8)
        .positionCentimeters(new Translation2d(29, 29))
        .drivePID(driveController)
        .pivotPID(pivotController)
        .wheelDiameterInches(4.0)
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(1, false))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(2, false))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(3, Rotation2d.fromDegrees(158.47), false))
      ),
      new SwerveModule(new SwerveModule.SwerveModuleConfig() // Front Left
        .driveGearRatio(6.75)
        .pivotGearRatio(12.8)
        .positionCentimeters(new Translation2d(-29, 29))
        .drivePID(driveController)
        .pivotPID(pivotController)
        .wheelDiameterInches(4.0)
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(4, false))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(5, false))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(6, Rotation2d.fromDegrees(0.26), false))
      ),
      new SwerveModule(new SwerveModule.SwerveModuleConfig() // Back Right
        .driveGearRatio(6.75)
        .pivotGearRatio(12.8)
        .positionCentimeters(new Translation2d(29, -29))
        .drivePID(driveController)
        .pivotPID(pivotController)
        .wheelDiameterInches(4.0)
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(7, false))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(8, false))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(9, Rotation2d.fromDegrees(-122.08), false))
      ),
      new SwerveModule(new SwerveModule.SwerveModuleConfig() // Back Left
        .driveGearRatio(6.75)
        .pivotGearRatio(12.8)
        .positionCentimeters(new Translation2d(-29, -29))
        .drivePID(driveController)
        .pivotPID(pivotController)
        .wheelDiameterInches(4.0)
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(10, true))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(11, false))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(12, Rotation2d.fromDegrees(123.05), false))
      )
    );
    this.controller = new XboxController(0);
  }

  @Override
  public void periodic() {
    this.swerveDrive.setSpeeds(
      MathUtil.applyDeadband(-this.controller.getLeftX(), 0.1), 
      MathUtil.applyDeadband(this.controller.getLeftY(), 0.1), 
      MathUtil.applyDeadband(this.controller.getRightX(), 0.1), 
      true
    );
    this.swerveDrive.update();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
  }
}
