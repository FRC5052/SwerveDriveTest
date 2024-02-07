// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveEncoder;
import frc.robot.swerve.SwerveIMU;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveMotor;
import frc.robot.swerve.SwerveDrive.HeadingControlMode;

public class SwerveDriveSubsystem extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private DoubleSupplier xAxis;
  private DoubleSupplier yAxis;
  private DoubleSupplier rAxis;
  /** Creates a new ExampleSubsystem. */
  public SwerveDriveSubsystem(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rAxis) {
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rAxis = rAxis;

    SwerveModule.SwerveModuleConfig cfg = new SwerveModule.SwerveModuleConfig()
      .driveGearRatio(6.75)
      .pivotGearRatio(12.8)
      .wheelDiameterInches(4.0)
    ;
    this.swerveDrive = new SwerveDrive(
      new Pose2d(), 
      new SwerveIMU.NavXSwerveIMU(),
      new SwerveModule(SwerveModule.SwerveModuleConfig.copyOf(cfg) // Front Left
        .positionInches(new Translation2d(-23.5, 23.5))
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(1, false, IdleMode.kCoast))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(2, false, IdleMode.kBrake))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(3, Rotation2d.fromDegrees(156.775), false))
      ),
      new SwerveModule(SwerveModule.SwerveModuleConfig.copyOf(cfg) // Front Right
        .positionInches(new Translation2d(23.5, 23.5))
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(7, false, IdleMode.kCoast))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(8, false, IdleMode.kBrake))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(9, Rotation2d.fromDegrees(-122.565), false))
      ),
      new SwerveModule(SwerveModule.SwerveModuleConfig.copyOf(cfg) // Back Right
        .positionInches(new Translation2d(23.5, -23.5))
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(10, true, IdleMode.kCoast))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(11, false, IdleMode.kBrake))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(12, Rotation2d.fromDegrees(126.33), false))
      ),
      new SwerveModule(SwerveModule.SwerveModuleConfig.copyOf(cfg) // Back Left
        .positionInches(new Translation2d(-23.5, -23.5))
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(5, false, IdleMode.kBrake))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(4, false, IdleMode.kCoast))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(6, Rotation2d.fromDegrees(-6.11), false))
      )
    );
    this.swerveDrive.setMaxDriveSpeed(7.5);
    this.swerveDrive.setMaxTurnSpeed(2.0);
    this.swerveDrive.setGlobalDrivePID(new PIDController(0.05, 0.0, 0.0));
    this.swerveDrive.setGlobalPivotPID(new PIDController(1.0, 0.0, 0.0));
    this.swerveDrive.setTurnPID(new PIDController(5.0, 0, 0.2));
    this.swerveDrive.setFieldCentric(true);
  }

  public void resetHeading() {
    this.swerveDrive.zeroHeading();
  }

  public void moveTo(Translation2d pos) {
    this.swerveDrive.moveTo(pos);
  }

  public void moveBy(Translation2d off) {
    this.swerveDrive.setFieldCentric(false);
    this.swerveDrive.moveBy(off);
  }

  public void cancelMove() {
    this.swerveDrive.cancelMove();
    this.swerveDrive.setFieldCentric(true);
  }

  public void setFullSpeed(boolean fullSpeed) {
    if (fullSpeed) {
      this.swerveDrive.setMaxDriveSpeed(7.5); // Full drive speed
      this.swerveDrive.setMaxTurnSpeed(2.0); // Full turn speed
    } else {
      this.swerveDrive.setMaxDriveSpeed(3.0); // Non-full drive speed
      this.swerveDrive.setMaxTurnSpeed(1.5); // Non-full turn speed
    }
  }

  @Override
  public void periodic() {
    this.swerveDrive.setSpeeds(
      Math.pow(MathUtil.applyDeadband(this.xAxis.getAsDouble(), 0.1), 3), 
      Math.pow(MathUtil.applyDeadband(this.yAxis.getAsDouble(), 0.1), 3), 
      Rotation2d.fromRotations(Math.pow(MathUtil.applyDeadband(this.rAxis.getAsDouble(), 0.1), 3) * 0.5),
      HeadingControlMode.kHeadingChange
    );
    this.swerveDrive.update();
  }
}
