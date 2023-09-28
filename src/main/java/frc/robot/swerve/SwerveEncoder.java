package frc.robot.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveEncoder {
    // d
    public double getAbsolutePosition();
    public double getPosition();
    public double getVelocity();
    public void setPosition(double position);
    public void setPositionToAbsolute();
    public void setOffset(Rotation2d offset);
    public Rotation2d getOffset();
    public void setReversed(boolean reverse);

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
    }
}
