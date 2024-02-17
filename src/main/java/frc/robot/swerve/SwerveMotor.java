package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Velocity;

public interface SwerveMotor {
    /** Returns the maximum speed in radians per second */
    public double maxSpeed(Velocity<Angle> unit);
    /** Gets the normalized throttle of the motor. */
    public double get();
    /** Sets the normalized throttle of the motor. */
    public void set(double speed);
    /** Returns whether the motor's output is reversed. */
    public boolean isReversed();
    /** Inverts the motor's output according to the given value. */
    public void setReversed(boolean reverse);
    /** Returns the motor's current power draw, in amps. */
    public double getCurrent(Current unit);
    /** Sets the motor's current limit to the given amp value. */
    public void setCurrentLimit(double limit, Current unit);
    /** Returns the current position reported by the motor since last being reset, in radians. */
    public double getPosition(Angle unit);
    /** Resets the position reported by the motor to the given radian value. */
    public void resetPosition(double position, Angle unit);
    /** Resets the position reported by the motor to zero. */
    public default void resetPosition() {
        this.resetPosition(0, Radians);
    }
    /** Returns the current velocity reported by the motor, in radians per second. */
    public double getVelocity(Velocity<Angle> unit);

    public static class CANSparkMaxSwerveMotor implements SwerveMotor {
        private CANSparkMax motor;

        public CANSparkMaxSwerveMotor(int id, boolean reversed, IdleMode idleMode) {
            this.motor = new CANSparkMax(id, MotorType.kBrushless);
            this.motor.setInverted(reversed);
            this.motor.setIdleMode(idleMode);
            this.motor.getEncoder().setPositionConversionFactor(1);
            this.motor.getEncoder().setVelocityConversionFactor(1);
        }

        @Override
        public double get() {
            return this.motor.get();
        }

        @Override
        public void set(double speed) {
            this.motor.set(speed);
        }

        @Override
        public void setReversed(boolean reverse) {
            this.motor.setInverted(reverse);
        }

        @Override
        public void setCurrentLimit(double limit, Current unit) {
            this.motor.setSmartCurrentLimit((int)Amps.convertFrom((double)limit, unit));
        }

        @Override
        public double getPosition(Angle unit) {
            return unit.convertFrom(this.motor.getEncoder().getPosition(), Rotations);
        }

        @Override
        public double getVelocity(Velocity<Angle> unit) {
            return unit.convertFrom(this.motor.getEncoder().getVelocity(), RPM);
        }

        @Override
        public boolean isReversed() {
            return this.motor.getEncoder().getInverted();
        }

        @Override
        public double getCurrent(Current unit) {
            return unit.convertFrom(this.motor.getOutputCurrent(), Amps);
        }

        @Override
        public void resetPosition(double position, Angle unit) {
            this.motor.getEncoder().setPosition(Rotations.convertFrom(position, unit));
        }

        @Override
        public double maxSpeed(Velocity<Angle> unit) {
            return unit.convertFrom(5820, RPM);
        }
    }
}
