// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeAndShooterSubsystem extends SubsystemBase {
  private static final double wristMotorMax = 20.0;
  private static final double elevatorMax = 30.0;

  private CANSparkMax lowerIntakeMotor;
  private CANSparkMax upperIntakeMotor;
  private CANSparkMax lowerShooterMotor;
  private CANSparkMax upperShooterMotor;
  private CANSparkMax wristMotor;
  private CANSparkMax elevatorMotor1, elevatorMotor2;
  private double targetWristPosition = wristMotorMax;
  private double targetElevatorPosition = 0.0;
  private Timer timer = new Timer();

  private ShootCommand shootCommand;
  private ChargedShootCommand chargedShootCommand;
  private HoldCommand holdCommand;
  private IntakeCommand intakeCommand;

  /** Creates a new ExampleSubsystem. */
  public IntakeAndShooterSubsystem() {
    this.shootCommand = new ShootCommand(this);
    this.chargedShootCommand = new ChargedShootCommand(this);
    this.holdCommand = new HoldCommand(this);
    this.intakeCommand = new IntakeCommand(this);

    this.lowerIntakeMotor = new CANSparkMax(14, MotorType.kBrushless);
    this.upperIntakeMotor = new CANSparkMax(13, MotorType.kBrushless);
    this.lowerIntakeMotor.setInverted(true);
    this.elevatorMotor1 = new CANSparkMax(18, MotorType.kBrushless);
    this.elevatorMotor2 = new CANSparkMax(19, MotorType.kBrushless);
    this.elevatorMotor2.follow(this.elevatorMotor1);
    this.lowerShooterMotor = new CANSparkMax(16, MotorType.kBrushless);
    this.upperShooterMotor = new CANSparkMax(15, MotorType.kBrushless);
    this.wristMotor = new CANSparkMax(17, MotorType.kBrushless);
    this.wristMotor.setInverted(true);
    this.wristMotor.setIdleMode(IdleMode.kBrake);
    this.timer.restart();
  }

  private void setWristTargetRaw(double normalPos) {
    this.targetWristPosition = MathUtil.clamp(normalPos, 0, 1) * wristMotorMax;
  }

  private double getWristTargetRaw() {
    return this.targetWristPosition / wristMotorMax;
  }

  private double getWristPosRaw() {
    return this.wristMotor.getEncoder().getPosition() / wristMotorMax;
  }

  public void setWristTarget(double value, Angle unit) {
    this.setWristTargetRaw(-(Degrees.convertFrom(value, unit) - 30) / 60);
  }

  public double getWristTarget(Angle unit) {
    return unit.convertFrom(-(this.getWristTargetRaw() * 60) + 30, Degrees);
  }

  public double getWristPosition(Angle unit) {
    return unit.convertFrom(-(this.getWristPosRaw() * 60) + 30, Degrees);
  }

  public void setElevatorTargetRaw(double target) {
    this.targetElevatorPosition = MathUtil.clamp(target, 0, 1) * elevatorMax;
  }

  public double getElevatorTargetRaw() {
    return this.targetElevatorPosition / elevatorMax;
  }

  public enum ShooterMode {
    
  }

  public void stopEverything() {
    this.upperShooterMotor.stopMotor();
    this.lowerShooterMotor.stopMotor();
    this.upperIntakeMotor.stopMotor();
    this.lowerShooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // -1.785
    // this.setElevatorTargetRaw((-Math.cos(0.25*Math.PI*this.timer.get()) / 1) + 0.5);
    // this.setElevatorTargetRaw(0.5);
    // System.out.println(this.targetElevatorPosition-this.elevatorMotor1.getEncoder().getPosition());
    // this.wristMotor.set((this.targetWristPosition-this.wristMotor.getEncoder().getPosition()) / wristMotorMax);

    // this.elevatorMotor1.set(((this.targetElevatorPosition-this.elevatorMotor1.getEncoder().getPosition()) / elevatorMax));
    // System.out.println(this.wristMotor.getEncoder().getPosition());
  }

  public static class ShootCommand extends Command {
    private IntakeAndShooterSubsystem subsystem;
    private Timer timer;
    private int stage;
    private double shooterSpeed;

    public ShootCommand(IntakeAndShooterSubsystem subsystem) {
      this.subsystem = subsystem;
      this.timer = new Timer();
      this.timer.start();
      addRequirements(this.subsystem);
    }

    @Override
    public void initialize() {
      this.timer.reset();
      stage = 1;
    }

    @Override
    public void execute() {
      switch (this.stage) {
        case 1:
          this.subsystem.lowerShooterMotor.set(-0.05);
          if (this.timer.get() > 0.05) {
            this.subsystem.lowerShooterMotor.stopMotor();
            this.stage++;
          }
          break;
        case 2:
          this.subsystem.upperShooterMotor.set(-this.shooterSpeed);
          if (this.timer.get() > 2.0) this.stage++;
          break;
        case 3:
          this.subsystem.lowerShooterMotor.set(this.shooterSpeed);
          if (this.timer.get() > 2.5) this.stage++;
          break;
        case 4:
          this.subsystem.lowerShooterMotor.stopMotor();
          this.subsystem.upperShooterMotor.stopMotor();
          this.stage = 0;
          break;
      }
    }

    @Override
    public boolean isFinished() {
        return this.stage <= 0; 
    }
  }

  public Command shoot(double speed) {
    this.shootCommand.shooterSpeed = MathUtil.clamp(speed, 0.0, 1.0);
    return this.shootCommand;
  }

  public static class ChargedShootCommand extends Command {
    private IntakeAndShooterSubsystem subsystem;
    private DoubleSupplier shooterSpeed;
    private int stage;
    private Timer timer;

    public ChargedShootCommand(IntakeAndShooterSubsystem subsystem) {
      this.subsystem = subsystem;
      this.timer = new Timer();
      this.timer.start();
      addRequirements(this.subsystem);
    }

    @Override
    public void initialize() {
      this.stage = 1;
    }

    @Override
    public void execute() {
      double speed = shooterSpeed.getAsDouble();
      if (speed < 0) {
        this.subsystem.upperIntakeMotor.set(-0.25);
        this.subsystem.lowerIntakeMotor.set(0.25);
      } else if (speed > 0) {
        this.subsystem.upperShooterMotor.set(-speed);
        this.subsystem.lowerShooterMotor.set(speed);
      } else {
        this.subsystem.upperShooterMotor.stopMotor();
        this.subsystem.lowerShooterMotor.stopMotor();
        this.subsystem.upperIntakeMotor.stopMotor();
        this.subsystem.lowerIntakeMotor.stopMotor();
      }
      // switch (this.stage) {
      //   case 1:
      //     if (shooterSpeed.getAsDouble() <= 0) {
      //       this.stage = 2;
      //       this.timer.restart();
      //       break;
      //     }
      //     this.subsystem.upperShooterMotor.set(-shooterSpeed.getAsDouble());
      //     this.subsystem.lowerShooterMotor.set(shooterSpeed.getAsDouble());
      //     break;
      //   case 2:
      //     this.subsystem.upperIntakeMotor.set(-0.4);
      //     this.subsystem.lowerIntakeMotor.set(0.4);
      //     if (this.timer.get() > 0.5) {
      //       this.stage = 3;
      //     }
      //     break;
      //   case 3:
      //     this.subsystem.upperShooterMotor.stopMotor();
      //     this.subsystem.lowerShooterMotor.stopMotor();
      //     this.subsystem.upperIntakeMotor.stopMotor();
      //     this.subsystem.lowerIntakeMotor.stopMotor();
      //     this.stage = 0;
      //     break;
      // }
    }

    @Override
    public boolean isFinished() {
      return this.stage == 0;
    }
  }

  public Command chargedShoot(DoubleSupplier speed) {
    this.chargedShootCommand.shooterSpeed = speed;
    return this.chargedShootCommand;
  }

  public static class HoldCommand extends Command {
    private IntakeAndShooterSubsystem subsystem;
    private Timer timer;
    private int stage;

    public HoldCommand(IntakeAndShooterSubsystem subsystem) {
      this.subsystem = subsystem;
      this.timer = new Timer();
      this.timer.start();
      addRequirements(this.subsystem);
    }

    @Override
    public void initialize() {
      this.timer.reset();
      stage = 1;
    }
    
    @Override
    public void execute() {
      switch (this.stage) {
        case 1:
          this.subsystem.lowerIntakeMotor.stopMotor();
          this.subsystem.upperIntakeMotor.stopMotor();
          this.subsystem.lowerShooterMotor.set(0.25);
          this.subsystem.upperShooterMotor.set(0.0);
          this.subsystem.upperIntakeMotor.set(-0.4);
          if (this.timer.get() > 0.75) {
            this.subsystem.lowerShooterMotor.stopMotor();
            this.subsystem.upperShooterMotor.stopMotor();
            this.subsystem.upperIntakeMotor.stopMotor();
            this.stage = 0;
          }
          break;
      }
    }

    @Override
    public boolean isFinished() {
        return this.stage <= 0; 
    }
  }

  public Command hold() {
    return this.holdCommand;
  }

  public static class  IntakeCommand extends Command {
    private IntakeAndShooterSubsystem subsystem;

    public IntakeCommand(IntakeAndShooterSubsystem subsystem) {
      this.subsystem = subsystem;
      addRequirements(this.subsystem);
    }

    @Override
    public void initialize() {
      this.subsystem.lowerIntakeMotor.set(0.4);
    }

    @Override
    public void end(boolean interrupted) {
      this.subsystem.lowerIntakeMotor.stopMotor();
    }
  }

  public Command intake(boolean thenHold) {
    if (thenHold) {
      return this.intakeCommand.andThen(this.holdCommand);
    } else {
      return this.intakeCommand;
    }
  }
}
