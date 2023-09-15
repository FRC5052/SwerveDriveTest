package frc.robot.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public interface SwerveEncoder {
    public double getAbsolutePosition();

    public class CANCoderSwerveEncoder implements SwerveEncoder {
        private CANCoder inst;

        public CANCoderSwerveEncoder(int id) {
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.unitString = 
        }
    }
}
