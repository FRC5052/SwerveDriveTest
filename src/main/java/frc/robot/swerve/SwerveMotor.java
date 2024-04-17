package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Velocity;

public abstract class SwerveMotor {
    /** Returns the name of this motor.
     * 
     * @return The name of this motor.
     */
    public String getMotorName() {
        return "Unknown";
    }
    /** Returns the maximum speed of this motor.
     * @param unit The unit of angular velocity to convert the measurement into.
     * @return The maximum speed that this motor can achieve.
     */
    public abstract double maxSpeed(Velocity<Angle> unit);
    /** Gets the normalized throttle of this motor. 
     * @return The normalized throttle that this motor reports, from -1 to 1.
    */
    public abstract double get();
    /** Sets the normalized throttle of this motor. 
     * @param speed The new normalized throttle of this motor, from -1 to 1.
    */
    public abstract void set(double speed);
    /** Returns whether this motor's output is reversed. 
     * @return true if this motor's output is reversed, false if it isn't.
    */
    public abstract boolean isReversed();
    /** Inverts this motor's output according to the given boolean value. 
     * @param reverse true to invert this motor's output, false if not.
    */
    public abstract void setReversed(boolean reverse);
    /** Returns this motor's current power draw. 
     * @param unit The unit of current to convert the measurement into.
     * @return The amount of current that this motor is reporting to draw.
    */
    public abstract double getCurrent(Current unit);
    /** Sets the motor's current limit to the given amp value. 
     * @param limit The new current limit to apply to this motor.
     * @param unit The unit of current to convert the limit into.
    */
    public abstract void setCurrentLimit(double limit, Current unit);
    /** Returns the current position reported by this motor since last being reset. 
     * @param unit The unit of angle to convert the measurement into.
     * @return The current position of the motor's shaft angle.
    */
    public abstract double getPosition(Angle unit);
    /** Sets the position reported by this motor to the given value. 
     * @param position The new angle to set the motor's shaft position to.
     * @param unit The unit of angle to convert the position from.
    */
    public abstract void setPosition(double position, Angle unit);
    /** Resets the position reported by the motor to zero. */
    public void resetPosition() {
        this.setPosition(0, Radians);
    }
    /** Returns the current velocity reported by the motor.
     * @param unit The unit of angular velocity to convert the measurement into.
     * @return The current angular velocity the motor's shaft is spinning at.
    */
    public abstract double getVelocity(Velocity<Angle> unit);

    public static class CANSparkMaxSwerveMotor extends SwerveMotor {
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
        public void setPosition(double position, Angle unit) {
            this.motor.getEncoder().setPosition(Rotations.convertFrom(position, unit));
        }

        @Override
        public double maxSpeed(Velocity<Angle> unit) {
            return unit.convertFrom(5820, RPM);
        }

        @Override
        public String getMotorName() {
            return "CAN Spark Max";
        }
    }
}
