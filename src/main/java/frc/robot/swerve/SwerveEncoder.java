package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.databind.JsonNode;
import com.revrobotics.CANSparkBase.IdleMode;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Objects;
import java.util.Optional;
import java.util.OptionalInt;

import org.json.simple.JSONObject;

import com.ctre.phoenix6.configs.CANcoderConfiguration;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.Velocity;

/** An interface for encoders used on swerve drive modules.
 * 
 */
public abstract class SwerveEncoder {
    public enum SwerveEncoderType {
        kCANCoder;

        public static SwerveEncoderType fromString(String str) {
            switch (str) {
                case "kCANCoder":
                    return kCANCoder;
                default:
                    return null;
            }
        }
    }

    public static SwerveEncoderBuilder getBuilder(SwerveEncoderType type) {
        switch (type) {
            case kCANCoder:
                return new CANCoderSwerveEncoder.Builder();
            default:
                return null;
        }
    }

    public static SwerveEncoderBuilder builderFromJSON(JsonNode json) {
        var builder = getBuilder(SwerveEncoderType.fromString(json.get("type").asText()));
        builder.fromJSON(json);
        return builder;
    }

    /** Returns the name of this encoder.
     * 
     * @return The name of this encoder.
     */
    public Optional<String> getEncoderName() {
        return Optional.empty();
    }

    /** Returns the absolute position of this encoder if it supports absolute positioning. Otherwise, this is the same as {@link #getPosition(Angle)}.
     * @param unit The angle unit to convert the measurement into.
     * @return The absolute position (or relative if this encoder doesn't support it) reported by this encoder.
    */
    public abstract double getAbsolutePosition(Angle unit);

    /** Returns the relative position of this encoder. 
     * @param unit The angle unit to convert the measurement into.
     * @return The relative position reported by this encoder.
    */
    public abstract double getPosition(Angle unit);

    /** Returns the velocity reported by this encoder. 
     * @param unit The angular velocity unit to convert the measurement into.
     * @return The angular velocity reported by this encoder.
    */
    public abstract double getVelocity(Velocity<Angle> unit);

    /** Sets the relative position that this encoder considers zero.
     * @param position The angle to set this encoder's zero to.
     * @param unit The angle unit to convert the measurement from.
    */
    public abstract void setPosition(double position, Angle unit);

    /** Sets the relative position of this encoder to match up with the absolute position, if it supports absolute position. Otherwise, nothing happens. */
    public void setPositionToAbsolute() {
        this.setPosition(this.getAbsolutePosition(Radians), Radians);
    }

    /** Sets the absolute offset of this encoder. 
     * @param offset The angle to offset the absolute position reported by this encoder by.
     * @param unit The angle unit to convert the measurement from.
    */
    public abstract void setOffset(double offset, Angle unit);

    /** Returns the absolute offset of this encoder. 
     * @param unit The angle unit to convert the measurement into.
     * @return The angle that the absolute position reported by this encoder is offset by.
    */
    public abstract double getOffset(Angle unit);

    /** Sets whether the output of this encoder is to be reversed. 
     * @param reverse Whether the encoder should measure clockwise as positive instead of negative.
    */
    public abstract void setReversed(boolean reverse);

    /** Returns whether this encoder supports absolute positioning. 
     * @return Whether the encoder will report unique values for {@link #getAbsolutePosition(Angle)}.
    */
    public abstract boolean isAbsolute();

    public static interface SwerveEncoderBuilder {
        public SwerveEncoderBuilder clone();
        public void fromJSON(JsonNode json);
        public SwerveEncoder build();
    }

    /** An implementation of {@link SwerveEncoder} for CTRE CANCoders. */
    public static class CANCoderSwerveEncoder extends SwerveEncoder {
        private CANcoder encoder;
        private CANcoderConfiguration config;

        private CANCoderSwerveEncoder(int id, boolean reverse, Measure<Angle> offset) {
            this.config = new CANcoderConfiguration();

            this.config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // Sets output to signed
            this.config.MagnetSensor.MagnetOffset = offset.in(Rotations);
            this.config.MagnetSensor.SensorDirection = reverse ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
            
            this.encoder = new CANcoder(id);
            this.encoder.getConfigurator().apply(this.config);
        }

        @Override
        public double getAbsolutePosition(Angle unit) {
            return unit.convertFrom(this.encoder.getAbsolutePosition().getValueAsDouble(), Rotations);
        }

        @Override
        public double getPosition(Angle unit) {
            return unit.convertFrom(this.encoder.getPosition().getValueAsDouble(), Rotations);
        }

        @Override
        public double getVelocity(Velocity<Angle> unit) {
            return unit.convertFrom(this.encoder.getVelocity().getValueAsDouble(), RotationsPerSecond);
        }

        @Override
        public void setPosition(double position, Angle unit) {
            this.encoder.setPosition(Rotations.convertFrom(position, unit));
        }

        @Override
        public void setOffset(double offset, Angle unit) {
            this.config.MagnetSensor.MagnetOffset = Rotations.convertFrom(offset, unit);
            this.encoder.getConfigurator().apply(this.config);
        }

        @Override
        public void setReversed(boolean reverse) {
            this.config.MagnetSensor.SensorDirection = reverse ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
            this.encoder.getConfigurator().apply(this.config);
        }

        @Override
        public double getOffset(Angle unit) {
            return unit.convertFrom(this.config.MagnetSensor.MagnetOffset, Rotations);
        }

        @Override
        public boolean isAbsolute() {
            return true;
        }

        @Override
        public Optional<String> getEncoderName() {
            return Optional.of("CTRE CanCoder");
        }

        public static class Builder implements SwerveEncoderBuilder {
            private OptionalInt id = OptionalInt.empty();
            private boolean reversed = false;
            private MutableMeasure<Angle> offset = MutableMeasure.zero(Radians);

            @Override
            public Builder clone() {
                Builder tmp = new Builder();
                tmp.id = this.id.isPresent() ? OptionalInt.of(this.id.getAsInt()) : OptionalInt.empty();
                tmp.reversed = this.reversed;
                tmp.offset = this.offset.mutableCopy();
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

            public Builder withOffset(double offset, Angle unit) {
                this.offset.mut_replace(offset, unit);
                return this;
            }

            public Builder withOffset(Measure<Angle> offset) {
                this.offset.mut_replace(offset);
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
                
                if (json.has("offset") && (json.get("offset").isObject() || json.get("offset").isDouble())) {
                    JsonNode json_inner = json.get("offset");
                    Angle unit = Degrees;
                    if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                        unit = Objects.requireNonNullElse(SwerveUtil.angleFromName(json_inner.get("unit").textValue()), unit);
                        this.withOffset(json_inner.get("value").doubleValue(), unit);
                    } else {
                        this.withOffset(json_inner.doubleValue(), unit);
                    }
                }
            }

            @Override
            public SwerveEncoder build() {
                if (this.id.isEmpty()) {
                    throw new IllegalStateException("ID field was empty");
                }
                return new CANCoderSwerveEncoder(this.id.getAsInt(), this.reversed, this.offset);
            }

        }
    }
}
