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
import com.revrobotics.CANSparkLowLevel.MotorType;

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
import frc.robot.swerve.SwerveMotor.SparkMaxSwerveMotor;
import frc.robot.swerve.SwerveEncoder.CANCoderSwerveEncoder;
import frc.robot.swerve.SwerveIMU.NavXSwerveIMU;

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

    SwerveModule.Builder module_cfg = new SwerveModule.Builder()
      .withDriveGearRatio(6.75)
      .withPivotGearRatio(12.8)
      .withWheelDiameter(0.1016, Meters);

    SparkMaxSwerveMotor.Builder motor_cfg = new SparkMaxSwerveMotor.Builder()
      .withIdleMode(IdleMode.kBrake) 
      .withMotorType(MotorType.kBrushless)
      .withCurrentLimit(40, Amps);

    CANCoderSwerveEncoder.Builder encoder_cfg = new CANCoderSwerveEncoder.Builder();

    this.swerveDrive = new SwerveDrive.Builder()
      .withIMU(new NavXSwerveIMU.Builder().withPort())
      .withModules(new SwerveModule.Builder[] {
        module_cfg.clone() // Front Right
          .withOffset(new Translation2d(23.5, -23.5), Inches)
          .withDriveMotor(motor_cfg.clone().withID(10))
          .withPivotMotor(motor_cfg.clone().withID(11))
          .withAbsoluteEncoder(encoder_cfg.clone().withID(12).withOffset(-240.64, Degrees)),
        module_cfg.clone() // Front Left
          .withOffset(new Translation2d(23.5, 23.5), Inches)
          .withDriveMotor(motor_cfg.clone().withID(1))
          .withPivotMotor(motor_cfg.clone().withID(2))
          .withAbsoluteEncoder(encoder_cfg.clone().withID(3).withOffset(-243.28, Degrees)),
        module_cfg.clone() // Back Right
          .withOffset(new Translation2d(-23.5, -23.5), Inches)
          .withDriveMotor(motor_cfg.clone().withID(7))
          .withPivotMotor(motor_cfg.clone().withID(8))
          .withAbsoluteEncoder(encoder_cfg.clone().withID(9).withOffset(36.29, Degrees)),
        module_cfg.clone() // Back Left
          .withOffset(new Translation2d(-23.5, 23.5), Inches)
          .withDriveMotor(motor_cfg.clone().withID(4))
          .withPivotMotor(motor_cfg.clone().withID(5))
          .withAbsoluteEncoder(encoder_cfg.clone().withID(6).withOffset(-34.1, Degrees))
      })
      .withHeadingPID(new PIDConstants(2.0, 0.0, 0.2))
      .withModuleDrivePID(new PIDConstants(0.5, 0.0, 0.0))
      .withModulePivotPID(new PIDConstants(1.0, 0.0, 0.0))
      .withFieldCentric(true)
      .build();

    this.setFullSpeed(true);
    double maxDistance = 0.0;
    for (int i = 0; i < this.swerveDrive.getNumSwerveModules(); i++) {
      SwerveModule module = this.swerveDrive.getSwerveModule(i);
      // module.getEncoder().setOffset(MathUtil.inputModulus(this.swerveDrive.getSwerveModule(i).getEncoder().getOffset(Degrees) - 90.0, -180, 180), Degrees);
      // module.getPivotMotor().setCurrentLimit(20, Amps);
      double distance = module.getModuleOffset().getNorm();
      if (maxDistance < distance) maxDistance = distance;
    }
    AutoBuilder.configureHolonomic(
        this.swerveDrive::getPose, 
        this.swerveDrive::setPose, 
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
          new PIDConstants(2.0, 0.0, 0.2),
          this.swerveDrive.getMaxDriveSpeed(MetersPerSecond), 
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
      this.swerveDrive.setMaxDriveAccel(6.0, MetersPerSecondPerSecond);
      this.swerveDrive.setMaxTurnAccel(1.5, RotationsPerSecond.per(Second));
    }
  }

  public void setFieldCentric(boolean fieldCentric) {
    this.swerveDrive.setIsFieldCentric(fieldCentric);
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
      // Normalize joystick vector.
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
        this.swerveDrive.setPose(new Pose2d(this.poseResetXFilter.calculate(aprilTagPose.get().getX()), this.poseResetYFilter.calculate(aprilTagPose.get().getY()), new Rotation2d(this.swerveDrive.getActualHeading(Radians))));
      }
      this.seeingAprilTag = aprilTagPose.isPresent();
    }
    this.swerveDrive.update();
    
  }
}
