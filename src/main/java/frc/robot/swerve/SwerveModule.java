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

    public void setDrivePID(PIDController controller, boolean clone) {
        this.cfg.driveController = clone ? new PIDController(controller.getP(), controller.getI(), controller.getD()) : controller;
        this.cfg.driveController.setIZone(Double.POSITIVE_INFINITY);
    }

    public void setDrivePID(PIDController controller) {
        this.setDrivePID(controller, false);
    }

    public void setPivotPID(PIDController controller, boolean clone) {
        this.cfg.pivotController = clone ? new PIDController(controller.getP(), controller.getI(), controller.getD()) : controller;
        this.cfg.pivotController.setIZone(Double.POSITIVE_INFINITY);
    }

    public void setPivotPID(PIDController controller) {
        this.setPivotPID(controller, false);
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
        return this.getActualSpeed() / this.cfg.driveMotor.maxSpeed();
    }

    // Returns the total distance traveled by the module's wheel in meters.
    public double getTotalDistance() {
        return (this.cfg.driveMotor.getPosition() / this.cfg.driveGearRatio) * this.cfg.getWheelRadius();
        // return this.cfg.driveMotor.getPosition() / (2*Math.PI);
    }

    public Translation2d getVelocityVector() {
        return new Translation2d(this.getActualSpeed(), this.getActualAngle());
    }

    public void update() {
        if (this.cfg.driveController != null) {
            double drive = this.cfg.driveController.calculate(this.getActualSpeed(), this.getStateSpeed());
            this.cfg.driveMotor.set(Double.isNaN(drive) ? 0.0 : drive);
        }
        if (this.cfg.pivotController != null) {
            double pivot = this.cfg.pivotController.calculate(this.getActualAngle().getRotations(), this.getStateAngle().getRotations());
            this.cfg.pivotMotor.set(Double.isNaN(pivot) ? 0.0 : pivot);
        }
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

        public double getWheelDiameter() {
            return this.wheelDiameter;
        }

        public double getWheelRadius() {
            return this.wheelDiameter / 2;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("stateSpeed", this::getStateSpeed, null);
        builder.addDoubleProperty("stateAngle", () -> this.getStateAngle().getDegrees(), null);
        builder.addDoubleProperty("actualSpeed", () -> this.getActualSpeed() * 100, null);
        builder.addDoubleProperty("actualAngle", () -> this.getActualAngle().getDegrees(), null);
        builder.addDoubleProperty("magnetOffset", () -> this.cfg.absoluteEncoder.getOffset().getDegrees(), (double val) -> this.cfg.absoluteEncoder.setOffset(Rotation2d.fromDegrees(val)));
        builder.addDoubleProperty("pControllerOutput", () -> {
            double driveDifference = this.getStateSpeed()-this.getActualSpeed(); 
            double driveSpeed = Math.abs(driveDifference) > 0.01 ? this.getActualSpeed()+Math.copySign(0.01, driveDifference) : this.getStateSpeed();
            return driveSpeed;
        }, null);
        builder.addDoubleProperty("pivotOutput", () -> this.cfg.pivotMotor.get(), null);
        builder.addDoubleProperty("driveOutput", () -> this.cfg.driveMotor.get(), null);
        builder.addDoubleProperty("totalDistance", this::getTotalDistance, null);
    }
}
