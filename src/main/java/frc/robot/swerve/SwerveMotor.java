package frc.robot.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public interface SwerveMotor {
    public double get();
    public void set(double speed);
    public void setReversed(boolean reverse);
    public void setCurrentLimit(int limit);
    public double getPosition();
    public double getVelocity();

    public class CANSparkMaxSwerveMotor implements SwerveMotor {
        private CANSparkMax motor;

        public CANSparkMaxSwerveMotor(int id, boolean reversed) {
            this.motor = new CANSparkMax(id, MotorType.kBrushless);
            this.motor.setInverted(reversed);
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
    }
}
