// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTags;
import frc.robot.Limelight;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveEncoder;
import frc.robot.swerve.SwerveIMU;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveMotor;
import frc.robot.swerve.SwerveDrive.HeadingControlMode;
import static frc.robot.swerve.SwerveIMU.*;

public class SwerveDriveSubsystem extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private DoubleSupplier xAxis;
  private DoubleSupplier yAxis;
  private DoubleSupplier rAxis;

  private Command pathfindingCommand;
  private boolean seeingAprilTag;
  private MedianFilter poseResetXFilter = new MedianFilter(10);
  private MedianFilter poseResetYFilter = new MedianFilter(10);
  /** Creates a new ExampleSubsystem. */
  public SwerveDriveSubsystem(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rAxis) {
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rAxis = rAxis;

    SwerveModule.SwerveModuleBuilder cfg = new SwerveModule.SwerveModuleBuilder()
      .driveGearRatio(6.75)
      .pivotGearRatio(12.8)
      .wheelDiameter(0.1016, Meters)
    ;
    this.swerveDrive = new SwerveDrive(
      new Pose2d(), 
      new SwerveIMU.NavXSwerveIMU(),
      SwerveModule.SwerveModuleBuilder.copyOf(cfg) // Front Right
        .position(new Translation2d(23.5, -23.5), Inches)
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(10, false, IdleMode.kBrake))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(11, false, IdleMode.kBrake))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(12, Rotation2d.fromDegrees(-240.64), false))
        .build(),
      SwerveModule.SwerveModuleBuilder.copyOf(cfg) // Front Left
        .position(new Translation2d(23.5, 23.5), Inches)
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(1, false, IdleMode.kBrake))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(2, false, IdleMode.kBrake))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(3, Rotation2d.fromDegrees(-243.28), false))
        .build(),
      SwerveModule.SwerveModuleBuilder.copyOf(cfg) // Back Right
        .position(new Translation2d(-23.5, -23.5), Inches)
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(7, false, IdleMode.kBrake))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(8, false, IdleMode.kBrake))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(9, Rotation2d.fromDegrees(36.29), false))
        .build(),
      SwerveModule.SwerveModuleBuilder.copyOf(cfg) // Back Left
        .position(new Translation2d(-23.5, 23.5), Inches)
        .driveMotor(new SwerveMotor.CANSparkMaxSwerveMotor(4, false, IdleMode.kBrake))
        .pivotMotor(new SwerveMotor.CANSparkMaxSwerveMotor(5, false, IdleMode.kBrake))
        .absoluteEncoder(new SwerveEncoder.CANCoderSwerveEncoder(6, Rotation2d.fromDegrees(-34.1), false))
        .build()
    );
    this.setFullSpeed(true);
    this.swerveDrive.setGlobalDrivePID(new PIDConstants(0.5, 0.0, 0.0));
    this.swerveDrive.setGlobalPivotPID(new PIDConstants(1.0, 0.0, 0.0));
    this.swerveDrive.setDriveController(
      new PIDConstants(0.8, 0.0, 0.0), 
      new PIDConstants(2.0, 0.0, 0.2)
    );
    this.swerveDrive.setFieldCentric(true);
    double maxDistance = 0.0;
    for (int i = 0; i < this.swerveDrive.getNumSwerveModules(); i++) {
      SwerveModule module = this.swerveDrive.getSwerveModule(i);
      // module.getEncoder().setOffset(MathUtil.inputModulus(this.swerveDrive.getSwerveModule(i).getEncoder().getOffset(Degrees) - 90.0, -180, 180), Degrees);
      module.getDriveMotor().setCurrentLimit(40, Amps);
      // module.getPivotMotor().setCurrentLimit(20, Amps);
      double distance = module.getModulePosition().getNorm();
      if (maxDistance < distance) maxDistance = distance;
    }
    AutoBuilder.configureHolonomic(
        this.swerveDrive::getPose, 
        this.swerveDrive::overridePosition, 
        this.swerveDrive::getActualSpeeds, 
        (ChassisSpeeds speeds) -> this.swerveDrive.drive(
          MathUtil.clamp(-speeds.vxMetersPerSecond/this.swerveDrive.getMaxDriveSpeed(MetersPerSecond), -1.0, 1.0),
          MathUtil.clamp(-speeds.vyMetersPerSecond/this.swerveDrive.getMaxDriveSpeed(MetersPerSecond), -1.0, 1.0),
          MathUtil.clamp(-speeds.omegaRadiansPerSecond/this.swerveDrive.getMaxTurnSpeed(RadiansPerSecond), -1.0, 1.0),
          HeadingControlMode.kSpeedOnly,
          Optional.of(false)
        ),
        new HolonomicPathFollowerConfig(
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(2.0, 0.0, 0.0),
          20.0, 
          maxDistance, 
          new ReplanningConfig()
        ),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, 
        this
    );
  }

  public void resetHeading() {
    this.swerveDrive.zeroHeading();
    // this.swerveDrive.overridePosition(new Pose2d());
  }

  public SwerveDrive getSwerveDrive() {
    return this.swerveDrive;
  }

  public void setFullSpeed(boolean fullSpeed) {
    if (fullSpeed) {
      this.swerveDrive.setMaxDriveSpeed(4.0, MetersPerSecond); // Full drive speed
      this.swerveDrive.setMaxTurnSpeed(0.75, RotationsPerSecond); // Full turn speed
      this.swerveDrive.setMaxDriveAccel(8.0, MetersPerSecondPerSecond);
      this.swerveDrive.setMaxTurnAccel(1.5, RotationsPerSecond.per(Second));
    } else {
      this.swerveDrive.setMaxDriveSpeed(2.0, MetersPerSecond); // Non-full drive speed
      this.swerveDrive.setMaxTurnSpeed(0.5, RotationsPerSecond); // Non-full turn speed
      this.swerveDrive.setMaxDriveAccel(8.0, MetersPerSecondPerSecond);
      this.swerveDrive.setMaxTurnAccel(1.5, RotationsPerSecond.per(Second));
    }
  }

  public void setFieldCentric(boolean fieldCentric) {
    this.swerveDrive.setFieldCentric(fieldCentric);
  }

  public void setTargetPose(Pose2d pose) {
    // this.swerveDrive.overrideTargetHeading(-this.swerveDrive.getPoseAngle(Radians), Radians);
    this.pathfindingCommand = AutoBuilder.pathfindToPose(
      pose, 
      new PathConstraints(
        this.swerveDrive.getMaxDriveSpeed(MetersPerSecond), 
        this.swerveDrive.getMaxDriveAccel(MetersPerSecondPerSecond), 
        this.swerveDrive.getMaxTurnSpeed(RadiansPerSecond), 
        this.swerveDrive.getMaxTurnAccel(RadiansPerSecond.per(Second))
      )
    );
    this.pathfindingCommand.schedule();
  }

  @Override
  public void periodic() {
    if (this.pathfindingCommand != null && this.pathfindingCommand.isScheduled()) {
      
    } else if (DriverStation.isTeleopEnabled()) {
      double x = Math.pow(MathUtil.applyDeadband(this.xAxis.getAsDouble(), 0.1), 3);
      double y = Math.pow(MathUtil.applyDeadband(this.yAxis.getAsDouble(), 0.1), 3);
      if (Math.abs(x*x + y*y) > 1.0) {
        double angle = Math.atan2(y, x);
        x = Math.cos(angle);
        y = Math.sin(angle);
      }
      this.swerveDrive.drive(
        x, 
        y, 
        Math.pow(MathUtil.applyDeadband(this.rAxis.getAsDouble(), 0.40), 3),
        HeadingControlMode.kHeadingChange,
        Optional.empty()
      );
    }
    if (DriverStation.isEnabled()) {
      Limelight.setRobotYaw(this.swerveDrive.getPoseAngle(Radians), this.swerveDrive.getActualSpeeds().omegaRadiansPerSecond, Radians, RadiansPerSecond);
      var aprilTagPose = Limelight.getFieldCentricRobotPose(Meters, true);
      if (aprilTagPose.isPresent() && this.seeingAprilTag) {
        this.swerveDrive.overridePosition(new Pose2d(this.poseResetXFilter.calculate(aprilTagPose.get().getX()), this.poseResetYFilter.calculate(aprilTagPose.get().getY()), new Rotation2d(this.swerveDrive.getActualHeading(Radians))));
      } else if (aprilTagPose.isPresent()) {
        // this.poseResetXFilter.reset();
        // this.poseResetYFilter.reset();
      }
      this.seeingAprilTag = aprilTagPose.isPresent();
    }
    this.swerveDrive.update();
    
  }
}
