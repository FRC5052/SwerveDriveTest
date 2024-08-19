package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.OptionalInt;

import com.fasterxml.jackson.databind.JsonNode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

public abstract class SwerveMotor {

    public enum SwerveMotorType {
        kSparkMax;

        public static SwerveMotorType fromString(String str) {
            switch (str) {
                case "kSparkMax":
                    return kSparkMax;
                default:
                    return null;
            }
        }
    }

    public static SwerveMotorBuilder getBuilder(SwerveMotorType type) {
        switch (type) {
            case kSparkMax:
                return new SparkMaxSwerveMotor.Builder();
            default:
                return null;
        }
    }

    public static SwerveMotorBuilder builderFromJSON(JsonNode json) {
        var builder = getBuilder(SwerveMotorType.fromString(json.get("type").asText()));
        builder.fromJSON(json);
        return builder;
    }

    /** Returns the name of this motor.
     * 
     * @return The name of this motor.
     */
    public Optional<String> getMotorName() {
        return Optional.empty();
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

    public static interface SwerveMotorBuilder {
        public SwerveMotorBuilder clone();
        public void fromJSON(JsonNode json);
        public SwerveMotor build();
    }

    public static class SparkMaxSwerveMotor extends SwerveMotor {
        private CANSparkMax motor;

        private SparkMaxSwerveMotor(int id, boolean reversed, IdleMode idleMode, MotorType type) {
            this.motor = new CANSparkMax(id, type);
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
        public Optional<String> getMotorName() {
            return Optional.of("CAN Spark Max");
        }

        public static class Builder implements SwerveMotorBuilder {
            private OptionalInt id = OptionalInt.empty();
            private boolean reversed = false;
            private Optional<IdleMode> idleMode = Optional.empty();
            private Optional<MotorType> motorType = Optional.empty();
            private Optional<MutableMeasure<Current>> currentLimit = Optional.empty();

            @Override
            public Builder clone() {
                Builder tmp = new Builder();
                tmp.id = this.id.isPresent() ? OptionalInt.of(this.id.getAsInt()) : OptionalInt.empty();
                tmp.idleMode = this.idleMode.isPresent() ? Optional.of(this.idleMode.get()) : Optional.empty();
                tmp.motorType = this.motorType.isPresent() ? Optional.of(this.motorType.get()) : Optional.empty();
                tmp.reversed = this.reversed;
                return tmp;
            }

            public Builder withID(int id) {
                this.id = OptionalInt.of(id);
                return this;
            }

            public Builder withReversed(boolean reversed) {
                this.reversed = reversed;
                return this;
            }

            public Builder withIdleMode(IdleMode idleMode) {
                this.idleMode = Optional.of(idleMode);
                return this;
            }

            public Builder withMotorType(MotorType motorType) {
                this.motorType = Optional.of(motorType);
                return this;
            }

            public Builder withCurrentLimit(double current, Current unit) {
                if (this.currentLimit.isEmpty()) {
                    this.currentLimit = Optional.of(MutableMeasure.ofRelativeUnits(current, unit));
                } else {
                    this.currentLimit.ifPresent((MutableMeasure<Current> measure) -> measure.mut_replace(current, unit));
                }
                return this;
            }

            @Override
            public void fromJSON(JsonNode json) {
                if (json.has("id") && json.get("id").isInt()) {
                    this.withID(json.get("id").intValue());
                }
                
                if (json.has("reversed") && json.get("reversed").isBoolean()) {
                    this.withReversed(json.get("reversed").booleanValue());
                }
                
                if (json.has("idleMode") && json.get("idleMode").isTextual()) {
                    String idleMode = json.get("idleMode").textValue();
                    switch (idleMode) {
                        case "kBrake":
                            this.withIdleMode(IdleMode.kBrake);
                            break;
                        case "kCoast":
                            this.withIdleMode(IdleMode.kCoast);
                            break;
                    }
                }
                
                if (json.has("motorType") && json.get("motorType").isTextual()) {
                    String motorType = json.get("motorType").textValue();
                    switch (motorType) {
                        case "kBrushed":
                            this.withMotorType(MotorType.kBrushed);
                            break;
                        case "kBrushless":
                            this.withMotorType(MotorType.kBrushless);
                            break;
                    }
                }
            }

            @Override
            public SwerveMotor build() {
                if (this.id.isEmpty()) {
                    throw new IllegalStateException("ID field was empty");
                }
                if (this.idleMode.isEmpty()) {
                    throw new IllegalStateException("IdleMode field was empty");
                }
                if (this.motorType.isEmpty()) {
                    throw new IllegalStateException("MotorType field was empty");
                }
                var tmp = new SparkMaxSwerveMotor(this.id.getAsInt(), this.reversed, this.idleMode.get(), this.motorType.get());
                this.currentLimit.ifPresent((MutableMeasure<Current> measure) -> tmp.setCurrentLimit(measure.magnitude(), measure.unit()));
                return tmp;
            }

        }
    }
}
