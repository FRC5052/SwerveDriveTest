package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;


import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveEncoder {
    /** Returns the absolute position of this encoder, in radians, if it supports absolute positioning. Otherwise, this is the same as getPosition. */
    public double getAbsolutePosition();
    /** Returns the relative position of this encoder, in radians. */
    public double getPosition();
    /** Returns the velocity reported by the encoder, in radians per second. */
    public double getVelocity();
    /** Sets the relative position of this encoder to the given radian value. */
    public void setPosition(double position);
    /** Sets the relative position of this encoder to match up with the absolute position, if it supports absolute position. Otherwise, nothing happens. */
    public void setPositionToAbsolute();
    /** Sets the absolute offset of this encoder. */
    public void setOffset(Rotation2d offset);
    /** Returns the absolute offset of this encoder. */
    public Rotation2d getOffset();
    /** Sets whether the output of the encoder are to be reversed. */
    public void setReversed(boolean reverse);
    /** Returns whether this encoder supports absolute positioning. */
    public boolean isAbsolute();

    public static class CANCoderSwerveEncoder implements SwerveEncoder {
        private CANcoder encoder;
        private CANcoderConfiguration config;
        private static final double conversionFactor = (2 * Math.PI);

        public CANCoderSwerveEncoder(int id, Rotation2d offset, boolean reverse) {
            this.config = new CANcoderConfiguration();

            this.config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // Sets output to signed
            this.config.MagnetSensor.MagnetOffset = offset.getRotations();
            this.config.MagnetSensor.SensorDirection = reverse ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
            
            this.encoder = new CANcoder(id);
            this.encoder.getConfigurator().apply(this.config);
        }

        @Override
        public double getAbsolutePosition() {
            return this.encoder.getAbsolutePosition().getValueAsDouble() * conversionFactor;
        }

        @Override
        public double getPosition() {
            return this.encoder.getPosition().getValueAsDouble() * conversionFactor;
        }

        @Override
        public double getVelocity() {
            return this.encoder.getVelocity().getValueAsDouble() * conversionFactor;
        }

        @Override
        public void setPosition(double position) {
            this.encoder.setPosition(position / conversionFactor);
        }

        @Override
        public void setPositionToAbsolute() {
            this.encoder.setPosition(this.encoder.getAbsolutePosition().getValueAsDouble());
        }

        @Override
        public void setOffset(Rotation2d offset) {
            this.config.MagnetSensor.MagnetOffset = offset.getRotations();
            this.encoder.getConfigurator().apply(this.config);
        }

        @Override
        public void setReversed(boolean reverse) {
            this.config.MagnetSensor.SensorDirection = reverse ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
            this.encoder.getConfigurator().apply(this.config);
        }

        @Override
        public Rotation2d getOffset() {
            return Rotation2d.fromRotations(this.config.MagnetSensor.MagnetOffset);
        }

        @Override
        public boolean isAbsolute() {
            return true;
        }
    }
}
