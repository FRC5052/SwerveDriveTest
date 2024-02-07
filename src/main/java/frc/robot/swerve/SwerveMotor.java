package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public interface SwerveMotor {
    /** Returns the maximum speed in radians per second */
    public double maxSpeed();
    /** Gets the normalized throttle of the motor. */
    public double get();
    /** Sets the normalized throttle of the motor. */
    public void set(double speed);
    /** Returns whether the motor's output is reversed. */
    public boolean isReversed();
    /** Inverts the motor's output according to the given value. */
    public void setReversed(boolean reverse);
    /** Returns the motor's current power draw, in amps. */
    public double getCurrent();
    /** Sets the motor's current limit to the given amp value. */
    public void setCurrentLimit(int limit);
    /** Returns the current position reported by the motor since last being reset, in radians. */
    public double getPosition();
    /** Resets the position reported by the motor to the given radian value. */
    public void resetPosition(double position);
    /** Resets the position reported by the motor to zero. */
    public default void resetPosition() {
        this.resetPosition(0);
    }
    /** Returns the current velocity reported by the motor, in radians per second. */
    public double getVelocity();

    public static class CANSparkMaxSwerveMotor implements SwerveMotor {
        private CANSparkMax motor;

        public CANSparkMaxSwerveMotor(int id, boolean reversed, IdleMode idleMode) {
            this.motor = new CANSparkMax(id, MotorType.kBrushless);
            this.motor.setInverted(reversed);
            this.motor.setIdleMode(idleMode);
            this.motor.getEncoder().setPositionConversionFactor(2 * Math.PI);
            this.motor.getEncoder().setVelocityConversionFactor((2 * Math.PI) / 60.0);
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
        public void setCurrentLimit(int limit) {
            this.motor.setSmartCurrentLimit(limit);
        }

        @Override
        public double getPosition() {
            return this.motor.getEncoder().getPosition();
        }

        @Override
        public double getVelocity() {
            return this.motor.getEncoder().getVelocity();
        }

        @Override
        public boolean isReversed() {
            return this.motor.getEncoder().getInverted();
        }

        @Override
        public double getCurrent() {
            return this.motor.getOutputCurrent();
        }

        @Override
        public void resetPosition(double position) {
            this.motor.getEncoder().setPosition(position);
        }

        @Override
        public double maxSpeed() {
            return Units.rotationsPerMinuteToRadiansPerSecond(5820);
        }
    }
}
