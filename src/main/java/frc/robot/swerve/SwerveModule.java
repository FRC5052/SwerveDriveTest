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

import java.util.Optional;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.ImmutableMeasure;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {
    private final SwerveMotor driveMotor, pivotMotor;
    private final SwerveEncoder absoluteEncoder;
    private Optional<PIDController> driveController, pivotController;
    private final Translation2d position;
    private final double driveGearRatio, pivotGearRatio;
    private final Measure<Distance> wheelDiameter;
    private boolean optimize = true;
    private SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    private SwerveModuleState actualState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    private SwerveModulePosition actualPosition = new SwerveModulePosition();
    private SwerveModulePosition actualDeltaPosition = new SwerveModulePosition();

    private Rotation2d actualAngle = new Rotation2d();

    /*
     * Constructs a
     */
    private SwerveModule(SwerveModuleBuilder cfg) {
        this.driveMotor = cfg.driveMotor;
        this.pivotMotor = cfg.pivotMotor;
        this.absoluteEncoder = cfg.absoluteEncoder;
        this.driveController = cfg.driveController.map((pid) -> new PIDController(pid.kP, pid.kI, pid.kD));;
        this.pivotController = cfg.pivotController.map((pid) -> new PIDController(pid.kP, pid.kI, pid.kD));;
        this.position = cfg.position;
        this.driveGearRatio = cfg.driveGearRatio;
        this.pivotGearRatio = cfg.pivotGearRatio;
        this.wheelDiameter = cfg.wheelDiameter.copy();
    }

    public SwerveMotor getDriveMotor() {
        return this.driveMotor;
    }

    public SwerveMotor getPivotMotor() {
        return this.pivotMotor;
    }

    public SwerveEncoder getEncoder() {
        return this.absoluteEncoder;
    }

    public void setDrivePID(Optional<PIDConstants> constants) {
        this.driveController = constants.map((pid) -> new PIDController(pid.kP, pid.kI, pid.kD));
    }

    public void setPivotPID(Optional<PIDConstants> constants) {
        this.pivotController = constants.map((pid) -> new PIDController(pid.kP, pid.kI, pid.kD));
        this.pivotController.ifPresent((pid) -> pid.enableContinuousInput(-0.5, 0.5));
    }

    private double getWheelRadius(Distance unit) {
        return this.wheelDiameter.in(unit) / 2.0;
    }

    private double getWheelCircumference(Distance unit) {
        return this.wheelDiameter.in(unit) * Math.PI;
    }

    public void setState(SwerveModuleState state) {
        if (this.optimize) {
            state = SwerveModuleState.optimize(state, this.getActualAngle());
        } 
        this.state = state;
    }

    public Translation2d getModulePosition() {
        return this.position;
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
        return MathUtil.applyDeadband(this.driveMotor.getVelocity(unit) / this.driveGearRatio, 1e-3);
    }

    public double getActualSpeed(Velocity<Distance> unit) {
        return this.getActualAngularSpeed(RadiansPerSecond) * this.getWheelRadius(unit.getUnit());
    }

    public double getActualNormalSpeed() {
        return this.getActualAngularSpeed(RadiansPerSecond) / this.driveMotor.maxSpeed(RadiansPerSecond);
    }

    // Returns the total distance traveled by the module's wheel in meters.
    public double getTotalDistance(Distance unit) {
        return -(this.driveMotor.getPosition(Radians) / this.driveGearRatio) * this.getWheelRadius(unit);
        // return this.driveMotor.getPosition() / (2*Math.PI);
    }

    public Translation2d getVelocityVector(Velocity<Distance> unit) {
        return new Translation2d(this.getActualSpeed(unit), this.getActualAngle());
    }

    public void update() {
        this.actualAngle = new Rotation2d(this.absoluteEncoder.getAbsolutePosition(Radians));
        this.actualState = new SwerveModuleState(this.getActualSpeed(MetersPerSecond), this.actualAngle);
        this.actualDeltaPosition = new SwerveModulePosition(this.actualPosition != null ? this.getTotalDistance(Meters) - this.actualPosition.distanceMeters : this.getTotalDistance(Meters), this.actualAngle);
        this.actualPosition = new SwerveModulePosition(this.getTotalDistance(Meters), this.actualAngle);

        if (this.driveMotor != null) {
            
            double drive = (((this.getStateSpeed(MetersPerSecond) * this.driveGearRatio) / this.getWheelRadius(Meters)) / this.driveMotor.maxSpeed(RadiansPerSecond));
            this.driveMotor.set(Double.isNaN(drive) ? 0.0 : drive);
        }
        if (this.pivotMotor != null) {
            double pivot = this.pivotController.isPresent() ? 
                this.pivotController.get().calculate(this.getActualAngle(Rotations), this.getStateAngle(Rotations)) 
                :
                this.getStateAngle(Rotations) - this.getActualAngle(Rotations);
            this.pivotMotor.set(Double.isNaN(pivot) ? 0.0 : pivot);
        }
    }

    public static class SwerveModuleBuilder {
        private SwerveMotor driveMotor, pivotMotor;
        private SwerveEncoder absoluteEncoder;
        private Optional<PIDConstants> driveController = Optional.empty(), pivotController = Optional.empty();
        private Translation2d position;
        private double driveGearRatio = 1.0, pivotGearRatio = 1.0;
        private Measure<Distance> wheelDiameter;
        private boolean optimize = true;

        public SwerveModuleBuilder() {
            
        }

        public static SwerveModuleBuilder copyOf(SwerveModuleBuilder src) {
            var dst = new SwerveModuleBuilder();
            if (src.driveController != null) dst.driveController = src.driveController.map((pid) -> new PIDConstants(pid.kP, pid.kI, pid.kP));
            else dst.driveController = null;
            if (src.pivotController != null) dst.pivotController = src.pivotController.map((pid) -> new PIDConstants(pid.kP, pid.kI, pid.kP));
            else dst.pivotController = null;
            dst.driveGearRatio = src.driveGearRatio;
            dst.pivotGearRatio = src.pivotGearRatio;
            dst.wheelDiameter = src.wheelDiameter.mutableCopy();
            dst.optimize = src.optimize;
            return dst;
        }

        public SwerveModule build() {
            if (
                this.driveMotor == null || 
                this.pivotMotor == null ||
                this.absoluteEncoder == null ||
                this.position == null ||
                this.wheelDiameter == null
            ) throw new IllegalStateException("Required fields missing when building SwerveModule!");
            return new SwerveModule(this);
        }

        public SwerveModuleBuilder driveMotor(SwerveMotor motor) {
            this.driveMotor = motor;
            return this;
        }

        public SwerveModuleBuilder pivotMotor(SwerveMotor motor) {
            this.pivotMotor = motor;
            return this;
        }

        public SwerveModuleBuilder absoluteEncoder(SwerveEncoder encoder) {
            this.absoluteEncoder = encoder;
            return this;
        }

        public SwerveModuleBuilder position(Translation2d position, Distance unit) {
            this.position = new Translation2d(unit.toBaseUnits(position.getX()), unit.toBaseUnits(position.getY()));
            return this;
        }

        public SwerveModuleBuilder wheelDiameter(double diameter, Distance unit) {
            this.wheelDiameter = ImmutableMeasure.ofRelativeUnits(diameter, unit);
            return this;
        }

        public SwerveModuleBuilder driveGearRatio(double ratio) {
            this.driveGearRatio = ratio;
            return this;
        }

        public SwerveModuleBuilder pivotGearRatio(double ratio) {
            this.pivotGearRatio = ratio;
            return this;
        }

        public SwerveModuleBuilder drivePID(Optional<PIDConstants> pid) {
            this.driveController = pid;
            return this;
        }

        public SwerveModuleBuilder pivotPID(Optional<PIDConstants> pid) {
            this.pivotController = pid;
            return this;
        }

        public SwerveModuleBuilder optimized(boolean enable) {
            this.optimize = enable;
            return this;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("stateSpeed", () -> this.getStateSpeed(MetersPerSecond) * 100, null);
        builder.addDoubleProperty("stateAngle", () -> this.getStateAngle(Degrees), null);
        builder.addDoubleProperty("actualSpeed", () -> this.getActualSpeed(MetersPerSecond) * 100, null);
        builder.addDoubleProperty("actualAngle", () -> this.getActualAngle(Degrees), null);
        builder.addDoubleProperty("magnetOffset", () -> this.absoluteEncoder.getOffset(Degrees), (double val) -> this.absoluteEncoder.setOffset(val, Degrees));
        builder.addDoubleProperty("pControllerOutput", () -> {
            double driveDifference = this.getStateSpeed(MetersPerSecond)-this.getActualSpeed(MetersPerSecond); 
            double driveSpeed = Math.abs(driveDifference) > 0.01 ? this.getActualSpeed(MetersPerSecond)+Math.copySign(0.01, driveDifference) : this.getStateSpeed(MetersPerSecond);
            return driveSpeed;
        }, null);
        builder.addDoubleProperty("pivotOutput", () -> this.pivotMotor.get(), null);
        builder.addDoubleProperty("driveOutput", () -> this.driveMotor.get(), null);
        builder.addDoubleProperty("totalDistance", () -> this.getTotalDistance(Meters), null);
        builder.addDoubleProperty("rawDistance", () -> MathUtil.inputModulus(this.getDriveMotor().getPosition(Degrees) / this.driveGearRatio, 0, 360), null);
        builder.addDoubleProperty("driveOutputCurrent", () -> this.getDriveMotor().getCurrent(Amps), null);
        builder.addDoubleProperty("pivotOutputCurrent", () -> this.getPivotMotor().getCurrent(Amps), null);
    }
}
