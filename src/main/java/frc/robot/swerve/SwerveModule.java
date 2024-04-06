package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {
    private SwerveModuleConfig cfg;
    private SwerveModuleState state;
    private SwerveModuleState actualState;
    private SwerveModulePosition actualPosition;
    private SwerveModulePosition actualDeltaPosition;

    private Rotation2d actualAngle;
    private Translation2d velocityVector;

    /*
     * Constructs a
     */
    public SwerveModule(SwerveModuleConfig cfg) {
        this.cfg = cfg;
        this.state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        this.actualAngle = new Rotation2d();
        this.velocityVector = new Translation2d();
    }

    public SwerveMotor getDriveMotor() {
        return this.cfg.driveMotor;
    }

    public SwerveMotor getPivotMotor() {
        return this.cfg.pivotMotor;
    }

    public SwerveEncoder getEncoder() {
        return this.cfg.absoluteEncoder;
    }

    public void setDrivePID(PIDController controller, boolean clone) {
        this.cfg.driveController = clone ? new PIDController(controller.getP(), controller.getI(), controller.getD()) : controller;
    }

    public void setDrivePID(PIDController controller) {
        this.setDrivePID(controller, false);
    }

    public void setPivotPID(PIDController controller, boolean clone) {
        this.cfg.pivotController = clone ? new PIDController(controller.getP(), controller.getI(), controller.getD()) : controller;
        this.cfg.pivotController.enableContinuousInput(-0.5, 0.5);
    }

    public void setPivotPID(PIDController controller) {
        this.setPivotPID(controller, false);
    }

    public void setState(SwerveModuleState state) {
        if (this.cfg.optimize) {
            state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(this.getActualAngle(Radians)));
        } 
        this.state = state;
    }

    public Translation2d getModulePosition() {
        return this.cfg.position;
    }

    public SwerveModuleState getState() {
        return this.state;
    }

    public SwerveModuleState getActualState() {
        return this.actualState;
    }

    public SwerveModulePosition getActualPosition() {
        return this.actualPosition;
    }

    public SwerveModulePosition getActualDeltaPosition() {
        return this.actualDeltaPosition;
    }
    

    /**
     * Returns the speed setpoint of this swerve module, in the given unit.
     * 
     * @param unit The unit to return the result in.
     * @return The speed setpoint of this swerve module.
     */
    public double getStateSpeed(Velocity<Distance> unit) {
        return unit.convertFrom(this.state.speedMetersPerSecond, MetersPerSecond);
    }

    /**
     * Returns the angle setpoint of this swerve module, in the given unit.
     * @param unit The unit to return the result in.
     * @return The speed setpoint of this swerve module.
     */
    public double getStateAngle(Angle unit) {
        return unit.convertFrom(this.state.angle.getRadians(), Radians);
    }

    public Rotation2d getStateAngle() {
        return this.state.angle;
    }

    public double getActualAngle(Angle unit) {
        return unit.convertFrom(this.actualAngle.getRadians(), Radians);
    }

    public Rotation2d getActualAngle() {
        return this.actualAngle;
    }

    // Returns the speed of the module in meters per second.

    public double getActualAngularSpeed(Velocity<Angle> unit) {
        return MathUtil.applyDeadband(this.cfg.driveMotor.getVelocity(unit) / this.cfg.driveGearRatio, 1e-3);
    }

    public double getActualSpeed(Velocity<Distance> unit) {
        return this.getActualAngularSpeed(RadiansPerSecond) * this.cfg.getWheelRadius(unit.getUnit());
    }

    public double getActualNormalSpeed() {
        return this.getActualAngularSpeed(RadiansPerSecond) / this.cfg.driveMotor.maxSpeed(RadiansPerSecond);
    }

    // Returns the total distance traveled by the module's wheel in meters.
    public double getTotalDistance(Distance unit) {
        return -(this.cfg.driveMotor.getPosition(Radians) / this.cfg.driveGearRatio) * this.cfg.getWheelRadius(unit);
        // return this.cfg.driveMotor.getPosition() / (2*Math.PI);
    }

    public Translation2d getVelocityVector(Velocity<Distance> unit) {
        return new Translation2d(this.getActualSpeed(unit), this.getActualAngle());
    }

    public void update() {
        this.actualAngle = new Rotation2d(this.cfg.absoluteEncoder.getAbsolutePosition(Radians));
        this.actualState = new SwerveModuleState(this.getActualSpeed(MetersPerSecond), this.actualAngle);
        this.actualDeltaPosition = new SwerveModulePosition(this.actualPosition != null ? this.getTotalDistance(Meters) - this.actualPosition.distanceMeters : this.getTotalDistance(Meters), this.actualAngle);
        this.actualPosition = new SwerveModulePosition(this.getTotalDistance(Meters), this.actualAngle);

        if (this.cfg.driveMotor != null) {
            
            double drive = (((this.getStateSpeed(MetersPerSecond) * this.cfg.driveGearRatio) / this.cfg.getWheelRadius(Meters)) / this.cfg.driveMotor.maxSpeed(RadiansPerSecond));
            this.cfg.driveMotor.set(Double.isNaN(drive) ? 0.0 : drive);
        }
        if (this.cfg.pivotMotor != null) {
            double pivot = this.cfg.pivotController != null ? 
                this.cfg.pivotController.calculate(this.getActualAngle(Rotations), this.getStateAngle(Rotations)) 
                :
                this.getStateAngle(Rotations) - this.getActualAngle(Rotations);
            this.cfg.pivotMotor.set(Double.isNaN(pivot) ? 0.0 : pivot);
        }
    }

    public static class SwerveModuleConfig {
        private SwerveMotor driveMotor, pivotMotor;
        private SwerveEncoder absoluteEncoder;
        private PIDController driveController, pivotController;
        private Translation2d position;
        private double driveGearRatio = 1.0, pivotGearRatio = 1.0;
        private MutableMeasure<Distance> wheelDiameter = MutableMeasure.zero(Meters);
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
            dst.wheelDiameter = src.wheelDiameter.mutableCopy();
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

        public SwerveModuleConfig position(Translation2d position, Distance unit) {
            this.position = new Translation2d(unit.toBaseUnits(position.getX()), unit.toBaseUnits(position.getY()));
            return this;
        }

        public SwerveModuleConfig wheelDiameter(double diameter, Distance unit) {
            this.wheelDiameter.mut_replace(diameter, unit);
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

        public double getWheelCircumference(Distance unit) {
            return this.wheelDiameter.in(unit) * Math.PI;
        }

        public double getWheelDiameter(Distance unit) {
            return this.wheelDiameter.in(unit);
        }

        public double getWheelRadius(Distance unit) {
            return this.wheelDiameter.in(unit) / 2;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("stateSpeed", () -> this.getStateSpeed(MetersPerSecond) * 100, null);
        builder.addDoubleProperty("stateAngle", () -> this.getStateAngle(Degrees), null);
        builder.addDoubleProperty("actualSpeed", () -> this.getActualSpeed(MetersPerSecond) * 100, null);
        builder.addDoubleProperty("actualAngle", () -> this.getActualAngle(Degrees), null);
        builder.addDoubleProperty("magnetOffset", () -> this.cfg.absoluteEncoder.getOffset(Degrees), (double val) -> this.cfg.absoluteEncoder.setOffset(val, Degrees));
        builder.addDoubleProperty("pControllerOutput", () -> {
            double driveDifference = this.getStateSpeed(MetersPerSecond)-this.getActualSpeed(MetersPerSecond); 
            double driveSpeed = Math.abs(driveDifference) > 0.01 ? this.getActualSpeed(MetersPerSecond)+Math.copySign(0.01, driveDifference) : this.getStateSpeed(MetersPerSecond);
            return driveSpeed;
        }, null);
        builder.addDoubleProperty("pivotOutput", () -> this.cfg.pivotMotor.get(), null);
        builder.addDoubleProperty("driveOutput", () -> this.cfg.driveMotor.get(), null);
        builder.addDoubleProperty("totalDistance", () -> this.getTotalDistance(Meters), null);
        builder.addDoubleProperty("rawDistance", () -> MathUtil.inputModulus(this.getDriveMotor().getPosition(Degrees) / this.cfg.driveGearRatio, 0, 360), null);
        builder.addDoubleProperty("driveOutputCurrent", () -> this.getDriveMotor().getCurrent(Amps), null);
        builder.addDoubleProperty("pivotOutputCurrent", () -> this.getPivotMotor().getCurrent(Amps), null);
    }
}
