package frc.robot.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public interface SwerveEncoder {
    // d
    public double getAbsolutePosition();
    public double getPosition();
    public double getVelocity();
    public void setPosition(double position);
    public void setPositionToAbsolute();
    public void setOffset(double offset);
    public void setReversed(boolean reverse);

    public class CANCoderSwerveEncoder implements SwerveEncoder {
        private CANCoder encoder;

        public CANCoderSwerveEncoder(int id, double offset, boolean reverse) {
            CANCoderConfiguration config = new CANCoderConfiguration();

            config.sensorCoefficient = (2 * Math.PI) / 4096.0; // Sets output range to 0 - 2pi
            config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180; // Sets output to signed
            config.magnetOffsetDegrees = offset * (180.0 / Math.PI);
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
        public void setOffset(double offset) {
            this.encoder.configMagnetOffset(offset * (180.0 / Math.PI));
        }

        @Override
        public void setReversed(boolean reverse) {
            this.encoder.configSensorDirection(reverse);
        }
    }
}
