package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    private SwerveModuleConfig cfg;
    private SwerveModuleState state;
    
    public SwerveModule(SwerveModuleConfig cfg) {
        this.cfg = cfg;
        this.state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    public void setState(SwerveModuleState state) {
        if (this.cfg.optimize) {
            state = SwerveModuleState.optimize(state, this.getActualAngle());
        } 
        this.state = state;
    }

    public Translation2d getModulePosition() {
        return this.cfg.position;
    }

    public SwerveModuleState getState() {
        return this.state;
    }

    public double getStateSpeed() {
        return this.state.speedMetersPerSecond;
    }

    public Rotation2d getStateAngle() {
        return this.state.angle;
    }

    public Rotation2d getActualAngle() {
        return Rotation2d.fromRadians(this.cfg.absoluteEncoder.getAbsolutePosition());
    }

    public double getActualSpeed() {
        return ((this.cfg.driveMotor.getVelocity() * this.cfg.driveGearRatio) / (2 * Math.PI)) * this.cfg.getWheelCircumference();
    }

    public void update() {
        this.cfg.driveMotor.set(this.cfg.driveController.calculate(this.getActualSpeed(), this.getStateSpeed()));
        this.cfg.pivotMotor.set(this.cfg.pivotController.calculate(this.getActualAngle().getRadians(), this.getStateAngle().getRadians()));
    }

    public class SwerveModuleConfig {
        private SwerveMotor driveMotor, pivotMotor;
        private SwerveEncoder absoluteEncoder;
        private PIDController driveController, pivotController;
        private Translation2d position;
        private double driveGearRatio = 1.0, pivotGearRatio = 1.0;
        private double wheelDiameter;
        private boolean optimize = true;

        public SwerveModuleConfig driveMotor(SwerveMotor motor) {
            this.driveMotor = motor;
            return this;
        }

        public SwerveModuleConfig pivotMotor(SwerveMotor motor) {
            this.pivotMotor = motor;
            return this;
        }

        public SwerveModuleConfig absoluteEncoder(SwerveEncoder encoder) {
            this.absoluteEncoder = encoder;
            return this;
        }

        public SwerveModuleConfig positionMeters(Translation2d position) {
            this.position = position;
            return this;
        }

        public SwerveModuleConfig positionFeet(Translation2d position) {
            this.positionMeters(new Translation2d(Units.feetToMeters(position.getX()), Units.feetToMeters(position.getY())));
            return this;
        }

        public SwerveModuleConfig positionCentimeters(Translation2d position) {
            this.positionMeters(new Translation2d(position.getX() / 100.0, position.getY() / 100.0));
            return this;
        }

        public SwerveModuleConfig positionInches(Translation2d position) {
            this.positionMeters(new Translation2d(Units.feetToMeters(position.getX() / 12.0), Units.feetToMeters(position.getY() / 12.0)));
            return this;
        }

        public SwerveModuleConfig driveGearRatio(double ratio) {
            this.driveGearRatio = ratio;
            return this;
        }

        public SwerveModuleConfig pivotGearRatio(double ratio) {
            this.pivotGearRatio = ratio;
            return this;
        }

        public SwerveModuleConfig drivePID(PIDController pid) {
            this.driveController = pid;
            return this;
        }

        public SwerveModuleConfig pivotPID(PIDController pid) {
            this.pivotController = pid;
            return this;
        }

        public SwerveModuleConfig optimized(boolean enable) {
            this.optimize = enable;
            return this;
        }

        public double getWheelCircumference() {
            return this.wheelDiameter * Math.PI;
        }
    }
}
