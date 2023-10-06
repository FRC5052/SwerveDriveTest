package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {
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

    // Returns the speed of the module in meters per second.
    public double getActualSpeed() {
        return MathUtil.applyDeadband(((this.cfg.driveMotor.getVelocity() / this.cfg.driveGearRatio) / (2 * Math.PI)) * this.cfg.getWheelCircumference(), 1e-3);
    }

    public double getActualNormalSpeed() {
        return this.getActualSpeed() / 200.0;
    }

    // Returns the total distance traveled by the module's wheel in meters.
    public double getTotalDistance() {
        return ((this.cfg.driveMotor.getPosition() / this.cfg.driveGearRatio) / (2 * Math.PI)) * this.cfg.getWheelCircumference();
    }

    public void update() {
        // double driveDifference = this.getStateSpeed()-this.getActualNormalSpeed(); 
        // double driveSpeed = driveDifference > 0.01 ? this.getActualNormalSpeed()+Math.copySign(0.01, driveDifference) : this.getStateSpeed();
        this.cfg.driveMotor.set((this.getStateSpeed()));
        this.cfg.pivotMotor.set(this.cfg.pivotController.calculate(this.getActualAngle().getRadians(), this.getStateAngle().getRadians()));
    }

    public static class SwerveModuleConfig {
        private SwerveMotor driveMotor, pivotMotor;
        private SwerveEncoder absoluteEncoder;
        private PIDController driveController, pivotController;
        private Translation2d position;
        private double driveGearRatio = 1.0, pivotGearRatio = 1.0;
        private double wheelDiameter = 0.0;
        private boolean optimize = true;

        public SwerveModuleConfig() {

        }

        public static SwerveModuleConfig copyOf(SwerveModuleConfig src) {
            var dst = new SwerveModuleConfig();
            if (src.driveController != null) dst.driveController = new PIDController(src.driveController.getP(), src.driveController.getI(), src.driveController.getD());
            else dst.driveController = null;
            if (src.pivotController != null) dst.pivotController = new PIDController(src.pivotController.getP(), src.pivotController.getI(), src.pivotController.getD());
            else dst.pivotController = null;
            dst.driveGearRatio = src.driveGearRatio;
            dst.pivotGearRatio = src.pivotGearRatio;
            dst.wheelDiameter = src.wheelDiameter;
            dst.optimize = src.optimize;
            return dst;
        }

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

        public SwerveModuleConfig wheelDiameterMeters(double diameter) {
            this.wheelDiameter = diameter;
            return this;
        }

        public SwerveModuleConfig wheelDiameterFeet(double diameter) {
            this.wheelDiameterMeters(Units.feetToMeters(diameter));
            return this;
        }

        public SwerveModuleConfig wheelDiameterCentimeters(double diameter) {
            this.wheelDiameterMeters(diameter / 100.0);
            return this;
        }

        public SwerveModuleConfig wheelDiameterInches(double diameter) {
            this.wheelDiameterMeters(Units.feetToMeters(diameter / 12.0));
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("stateSpeed", this::getStateSpeed, null);
        builder.addDoubleProperty("stateAngle", () -> this.getStateAngle().getDegrees(), null);
        builder.addDoubleProperty("actualSpeed", () -> this.getActualSpeed() / 2.0, null);
        builder.addDoubleProperty("actualAngle", () -> this.getActualAngle().getDegrees(), null);
        builder.addDoubleProperty("magnetOffset", () -> this.cfg.absoluteEncoder.getOffset().getDegrees(), (double val) -> this.cfg.absoluteEncoder.setOffset(Rotation2d.fromDegrees(val)));
        builder.addDoubleProperty("totalDistance", this::getTotalDistance, null);
    }
}
