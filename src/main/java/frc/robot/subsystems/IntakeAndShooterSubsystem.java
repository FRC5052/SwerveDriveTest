// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeAndShooterSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private CANSparkMax indexerMotor;
  private CANSparkMax shooterMotor;
  private CANSparkMax armMotor;
  private CANSparkMax pitchMotor;
  private CANSparkMax yawMotor;
  private double shooterSpeed;
  private double intakeSpeed;
  private double indexerSpeed;
  private int armHoldTimer = 0;
  private double pitchTarget = 0.0;
  private double yawTarget = 0.0;
  /** Creates a new ExampleSubsystem. */
  public IntakeAndShooterSubsystem() {
    this.armMotor = new CANSparkMax(13, MotorType.kBrushless);
    this.intakeMotor = new CANSparkMax(14, MotorType.kBrushless);
    this.indexerMotor = new CANSparkMax(15, MotorType.kBrushless);
    this.shooterMotor = new CANSparkMax(16, MotorType.kBrushless);
    this.pitchMotor = new CANSparkMax(17, MotorType.kBrushless);
    this.yawMotor = new CANSparkMax(18, MotorType.kBrushless);
  }

  public void retractArm() {
    this.armHoldTimer = 0;
  }

  public void extendArm(double time) {
    this.armHoldTimer = (int)(time / Robot.kDefaultPeriod);
  }

  public boolean isArmExtended() {
    return this.armMotor.getEncoder().getPosition() < -0.8;
  }

  public boolean isArmRetracted() {
    return this.armMotor.getEncoder().getPosition() > -0.5;
  }

  public void setShooterSpeed(double speed) {
    this.shooterSpeed = speed;
    System.out.println(this.shooterSpeed);
  }

  public double getShooterSpeed() {
    return this.shooterSpeed;
  }

  public void setIntakeSpeed(double speed) {
    this.intakeSpeed = speed;
  }

  public double getIntakeSpeed() {
    return this.intakeSpeed;
  }

  public void setIndexerSpeed(double speed) {
    this.indexerSpeed = speed;
  }

  public double getIndexerSpeed() {
    return this.indexerSpeed;
  }

  public Rotation2d getTargetedPitch() {
    return Rotation2d.fromDegrees((this.pitchTarget * (30.0/8.5)) + 25.0);
  }

  public Rotation2d getActualPitch() {
    return Rotation2d.fromDegrees((this.pitchMotor.getEncoder().getPosition() * (30.0/8.5)) + 25.0);
  }

  public Rotation2d getTargetedYaw() {
    return Rotation2d.fromDegrees(this.yawTarget * (180.0/21.0));
  }

  public Rotation2d getActualYaw() { 
    return Rotation2d.fromDegrees(this.yawMotor.getEncoder().getPosition()  * (180.0/21.0));
  }

  public void pitchToRaw(double value) {
    this.pitchTarget = MathUtil.clamp(value, 0.0, 8.5);
  }

  public void pitchTo(Rotation2d angle) {
    this.pitchToRaw(((90.0 - angle.getDegrees()) - 25.0) * (8.5/30.0));
  }

  public void pitchBy(Rotation2d angle) {
    this.pitchToRaw(this.pitchTarget + (angle.getDegrees() * (8.5/30.0)));
  }

  public void yawToRaw(double value) {
    this.yawTarget = MathUtil.clamp(value, -18.5, 21.0);
  }

  public void yawTo(Rotation2d angle) {
    this.yawToRaw(angle.getDegrees() * (21.0/180.0));
  }

  public void yawBy(Rotation2d angle) {
    this.yawToRaw(this.yawTarget + (angle.getDegrees() * (21.0/180.0)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.intakeMotor.set(this.intakeSpeed * -0.20);
    this.indexerMotor.set(this.indexerSpeed * 0.30);
    this.shooterMotor.set(this.shooterSpeed);
    if (this.armHoldTimer > 0) {
      this.armMotor.set(-0.25);
      this.armHoldTimer--;
    } else if (this.isArmExtended()) { 
      this.armMotor.set(0.25);
    } else {
      this.armMotor.stopMotor();
    }
    this.pitchMotor.set(MathUtil.clamp((this.pitchTarget-this.pitchMotor.getEncoder().getPosition()) * 0.1, -0.1, 0.1));
    this.yawMotor.set(MathUtil.clamp((this.yawTarget-this.yawMotor.getEncoder().getPosition()) * 0.15, -0.1, 0.1));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static class IntakeBallCommand extends Command {
    private int stage = 0;
    private long ticks = 0;
    private IntakeAndShooterSubsystem subsystem;

    public IntakeBallCommand(IntakeAndShooterSubsystem subsystem) {
      this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
      stage = 0;
      ticks = 0;
    }
    
    @Override
    public void execute() {
      switch (stage) {
        case 0: // Extend arm and turn on intake.
          subsystem.extendArm(0.5);
          subsystem.setIntakeSpeed(1.0);
          stage = 1;
          break;
        case 1: // Wait for arm to extend.
          if (ticks > (int)(1.5 / Robot.kDefaultPeriod)) stage = 2;
          else ticks++;
          break;
        case 2: 
          subsystem.setIntakeSpeed(0.0);
          stage = -1;
          return;
      }
    }

    @Override
    public boolean isFinished() {
      return stage == -1;
    }
  }

  public static class ShootBallCommand extends Command {
    private int stage = 0;
    private long ticks = 0;
    private IntakeAndShooterSubsystem subsystem;

    public ShootBallCommand(IntakeAndShooterSubsystem subsystem) {
      this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
      stage = 0;
      ticks = 0;
    }
    
    @Override
    public void execute() {
      switch (stage) {
        case 0: // Extend arm and turn on intake.
          subsystem.setIntakeSpeed(1.0);
          subsystem.setIndexerSpeed(1.0);
          stage = 1;
          break;
        case 1: // Wait for arm to extend.
          if (ticks > (int)(1.0 / Robot.kDefaultPeriod)) stage = 2;
          else ticks++;
          break;
        case 2: 
          subsystem.setIndexerSpeed(0.0);
          subsystem.setIntakeSpeed(0.0);
          ticks = 0;
          stage = 3;
          break;
        case 3: 
          if (ticks > (int)(0.3 / Robot.kDefaultPeriod)) stage = 4;
          else ticks++;
          break;
        case 4:
          subsystem.setShooterSpeed(0.0);
          stage = -1;
          break;
      }
    }

    @Override
    public boolean isFinished() {
      return stage == -1;
    }
  }

  public Command getIntakeBallCommand() {
    return new IntakeBallCommand(this);
  }

  public Command getShootBallCommand() {
    return new ShootBallCommand(this);
  }
}
