package frc.robot.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

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
    /** Returns if this encoder supports absolute positioning. */
    public boolean isAbsolute();

    public static class CANCoderSwerveEncoder implements SwerveEncoder {
        private CANCoder encoder;

        public CANCoderSwerveEncoder(int id, Rotation2d offset, boolean reverse) {
            CANCoderConfiguration config = new CANCoderConfiguration();

            config.sensorCoefficient = (2 * Math.PI) / 4096.0; // Sets output range to 0 - 2pi
            config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180; // Sets output to signed
            config.magnetOffsetDegrees = offset.getDegrees();
            config.sensorDirection = reverse;
            
            this.encoder = new CANCoder(id);
            this.encoder.configAllSettings(config);
        }

        @Override
        public double getAbsolutePosition() {
            return this.encoder.getAbsolutePosition();
        }

        @Override
        public double getPosition() {
            return this.encoder.getPosition();
        }

        @Override
        public double getVelocity() {
            return this.encoder.getVelocity();
        }

        @Override
        public void setPosition(double position) {
            this.encoder.setPosition(position);
        }

        @Override
        public void setPositionToAbsolute() {
            this.encoder.setPositionToAbsolute();
        }

        @Override
        public void setOffset(Rotation2d offset) {
            this.encoder.configMagnetOffset(offset.getDegrees());
        }

        @Override
        public void setReversed(boolean reverse) {
            this.encoder.configSensorDirection(reverse);
        }

        @Override
        public Rotation2d getOffset() {
            return Rotation2d.fromDegrees(this.encoder.configGetMagnetOffset());
        }

        @Override
        public boolean isAbsolute() {
            return true;
        }
    }
}
