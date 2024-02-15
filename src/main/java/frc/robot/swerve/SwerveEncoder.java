package frc.robot.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Velocity;

/** An interface for encoders used on swerve drive modules.
 * 
 */
public interface SwerveEncoder {

    /** Returns the absolute position of this encoder if it supports absolute positioning. Otherwise, this is the same as {@link #getPosition(Angle)}.
     * @param unit The angle unit to convert the measurement into.
     * @return The absolute position (or relative if this encoder doesn't support it) reported by this encoder.
    */
    public double getAbsolutePosition(Angle unit);

    /** Returns the relative position of this encoder. 
     * @param unit The angle unit to convert the measurement into.
     * @return The relative position reported by this encoder.
    */
    public double getPosition(Angle unit);

    /** Returns the velocity reported by this encoder. 
     * @param unit The angular velocity unit to convert the measurement into.
     * @return The angular velocity reported by this encoder.
    */
    public double getVelocity(Velocity<Angle> unit);

    /** Sets the relative position that this encoder considers zero.
     * @param position The angle to set this encoder's zero to.
     * @param unit The angle unit to convert the measurement from.
    */
    public void setPosition(double position, Angle unit);

    /** Sets the relative position of this encoder to match up with the absolute position, if it supports absolute position. Otherwise, nothing happens. */
    public void setPositionToAbsolute();

    /** Sets the absolute offset of this encoder. 
     * @param offset The angle to offset the absolute position reported by this encoder by.
     * @param unit The angle unit to convert the measurement from.
    */
    public void setOffset(double offset, Angle unit);

    /** Returns the absolute offset of this encoder. 
     * @param unit The angle unit to convert the measurement into.
     * @return The angle that the absolute position reported by this encoder is offset by.
    */
    public double getOffset(Angle unit);

    /** Sets whether the output of this encoder is to be reversed. 
     * @param reverse Whether the encoder should measure clockwise as positive instead of negative.
    */
    public void setReversed(boolean reverse);

    /** Returns whether this encoder supports absolute positioning. 
     * @return Whether the encoder will report unique values for {@link #getAbsolutePosition(Angle)}.
    */
    public boolean isAbsolute();

    /** An implementation of {@link SwerveEncoder} for CTRE CANCoders. */
    public static class CANCoderSwerveEncoder implements SwerveEncoder {
        private CANcoder encoder;
        private CANcoderConfiguration config;

        public CANCoderSwerveEncoder(int id, Rotation2d offset, boolean reverse) {
            this.config = new CANcoderConfiguration();

            this.config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // Sets output to signed
            this.config.MagnetSensor.MagnetOffset = offset.getRotations();
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
        public void setPositionToAbsolute() {
            this.encoder.setPosition(this.encoder.getAbsolutePosition().getValueAsDouble());
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
    }
}
