package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static edu.wpi.first.units.Units.*;

import java.util.Objects;
import java.util.Optional;

import com.fasterxml.jackson.databind.JsonNode;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {
    private final SwerveMotor driveMotor, pivotMotor;
    private final SwerveEncoder absoluteEncoder;
    private Optional<PIDController> driveController, pivotController;
    private final Translation2d position;
    private final double driveGearRatio, pivotGearRatio;
    private final double wheelDiameter;
    private boolean optimize = true;
    private SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    private SwerveModuleState actualState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    private SwerveModulePosition actualPosition = new SwerveModulePosition();
    private SwerveModulePosition actualDeltaPosition = new SwerveModulePosition();

    private Rotation2d actualAngle = new Rotation2d();

    private SwerveModule(Builder cfg) {
        this.driveMotor = cfg.driveMotor.get().build();
        this.pivotMotor = cfg.pivotMotor.get().build();
        this.absoluteEncoder = cfg.absoluteEncoder.get().build();
        this.driveController = cfg.drivePID.map((pid) -> new PIDController(pid.kP, pid.kI, pid.kD));;
        this.pivotController = cfg.pivotPID.map((pid) -> new PIDController(pid.kP, pid.kI, pid.kD));;
        this.position = cfg.offset;
        this.driveGearRatio = cfg.driveGearRatio;
        this.pivotGearRatio = cfg.pivotGearRatio;
        this.wheelDiameter = cfg.wheelDiameter;
    }

    /**
     * Returns this swerve module's drive motor.
     * @return A SwerveMotor representing this swerve module's drive motor.
     */
    public SwerveMotor getDriveMotor() {
        return this.driveMotor;
    }

    /**
     * Returns this swerve module's pivot motor.
     * @return A SwerveMotor representing this swerve module's pivot motor.
     */
    public SwerveMotor getPivotMotor() {
        return this.pivotMotor;
    }

    /**
     * Returns this swerve module's absolute encoder.
     * @return A SwerveMotor representing this swerve module's absolute encoder.
     */
    public SwerveEncoder getEncoder() {
        return this.absoluteEncoder;
    }

    /**
     * Configures this swerve module's drive PID constants.
     * @param constants The PID constants to apply.
     */
    public void setDrivePID(Optional<PIDConstants> constants) {
        this.driveController = constants.map((pid) -> new PIDController(pid.kP, pid.kI, pid.kD));
    }

    /**
     * Configures this swerve module's drive PID constants.
     * @param constants The PID constants to apply.
     */
    public void setPivotPID(Optional<PIDConstants> constants) {
        this.pivotController = constants.map((pid) -> new PIDController(pid.kP, pid.kI, pid.kD));
        this.pivotController.ifPresent((pid) -> pid.enableContinuousInput(-0.5, 0.5)); // DO NOT CHANGE THIS TO RADIANS
    }

    /**
     * Returns the diameter of the swerve module's wheel.
     * @param unit The unit of length to use to convert the value.
     * @return The diameter of the swerve module's wheel, in the given units.
     */
    public double getWheelDiameter(Distance unit) {
        return unit.convertFrom(this.wheelDiameter, Meters);
    }

    /**
     * Returns the radius of the swerve module's wheel.
     * @param unit The unit of length to use to convert the value.
     * @return The radius of the swerve module's wheel, in the given units.
     */
    public double getWheelRadius(Distance unit) {
        return this.getWheelDiameter(unit) / 2.0;
    }

    /**
     * Returns the circumfrence of the swerve module's wheel.
     * @param unit The unit of length to use to convert the value.
     * @return The circumfrence of the swerve module's wheel, in the given units.
     */
    public double getWheelCircumference(Distance unit) {
        return this.getWheelDiameter(unit) * Math.PI;
    }

    /**
     * Sets the module state to be targeted by this swerve module.
     * @param state The new module state to apply.
     */
    public void setTargetState(SwerveModuleState state) {
        if (this.optimize) {
            state = SwerveModuleState.optimize(state, this.getActualAngle());
        } 
        this.state = state;
    }

    /**
     * Returns the offset of this swerve module from the center of the swerve drive.
     * @return A Translation2d representing the offset.
     */
    public Translation2d getModuleOffset() {
        return this.position;
    }

    /**
     * Returns this swerve module's targeted module state.
     * @return A SwerveModuleState representing the targeted module state.
     */
    public SwerveModuleState getTargetState() {
        return this.state;
    }

    /**
     * Returns this swerve module's measured module state.
     * @return A SwerveModuleState representing the measured module state.
     */
    public SwerveModuleState getActualState() {
        return this.actualState;
    }

    /**
     * Returns this swerve module's current position.
     * @return A SwerveModulePositon representing the module's position.
     */
    public SwerveModulePosition getActualPosition() {
        return this.actualPosition;
    }

    /**
     * Returns this swerve module's last delta position (current position - last position).
     * @return A SwerveModulePositon representing the module's delta position.
     */
    public SwerveModulePosition getActualDeltaPosition() {
        return this.actualDeltaPosition;
    }

    /**
     * Returns the speed setpoint of this swerve module.
     * @param unit The unit of velocity to use to convert the value.
     * @return The speed setpoint of this swerve module, in the given units.
     */
    public double getStateSpeed(Velocity<Distance> unit) {
        return unit.convertFrom(this.state.speedMetersPerSecond, MetersPerSecond);
    }

    /**
     * Returns the angle setpoint of this swerve module.
     * @return The angle setpoint of this swerve module.
     */
    public Rotation2d getStateAngle() {
        return this.state.angle;
    }

    /**
     * Returns the angle setpoint of this swerve module.
     * @param unit The unit of velocity to use to convert the value.
     * @return The angle setpoint of this swerve module, in the given units.
     */
    public double getStateAngle(Angle unit) {
        return unit.convertFrom(this.state.angle.getRadians(), Radians);
    }

    /**
     * Returns the measured angle of this swerve module.
     * @return The measured angle of this swerve module.
     */
    public Rotation2d getActualAngle() {
        return this.actualAngle;
    }

    /**
     * Returns the measured angle of this swerve module.
     * @param unit The unit of angle to use to convert the value.
     * @return The measured angle of this swerve module, in the given units.
     */
    public double getActualAngle(Angle unit) {
        return unit.convertFrom(this.actualAngle.getRadians(), Radians);
    }
    
    /**
     * Returns the measured angular velocity of this swerve module.
     * @param unit The unit of angular velocity to use to convert the value.
     * @return The measured angular velocity of this swerve module, in the given units.
     */
    public double getMeasuredAngularVelocity(Velocity<Angle> unit) {
        return MathUtil.applyDeadband(this.driveMotor.getVelocity(unit) / this.driveGearRatio, 1e-3);
    }

    /**
     * Returns the measured velocity of this swerve module.
     * @param unit The unit of velocity to use to convert the value.
     * @return The measured velocity of this swerve module, in the given units.
     */
    public double getMeasuredVelocity(Velocity<Distance> unit) {
        return this.getMeasuredAngularVelocity(RadiansPerSecond) * this.getWheelRadius(unit.getUnit());
    }

    /**
     * Returns the measured velocity of this swerve module as a normalized value between -1 and 1.
     * @return The measured velocity normal of this swerve module.
     */
    public double getMeasuredVelocityNormal() {
        return this.getMeasuredAngularVelocity(RadiansPerSecond) / this.driveMotor.maxSpeed(RadiansPerSecond);
    }

    /**
     * Returns the total distance traveled by this swerve module's wheel.
     * @param unit The unit of distance to use to convert the value.
     * @return The total distance this swerve module's wheel has traveled, in the given units.
     */
    public double getTotalDistance(Distance unit) {
        return -(this.driveMotor.getPosition(Radians) / this.driveGearRatio) * this.getWheelRadius(unit);
        // return this.driveMotor.getPosition() / (2*Math.PI);
    }

    /**
     * Updates the internal logic of this swerve module.
     */
    public void update() {
        this.actualAngle = new Rotation2d(this.absoluteEncoder.getAbsolutePosition(Radians));
        this.actualState = new SwerveModuleState(this.getMeasuredVelocity(MetersPerSecond), this.actualAngle);
        this.actualDeltaPosition = new SwerveModulePosition(this.actualPosition != null ? this.getTotalDistance(Meters) - this.actualPosition.distanceMeters : this.getTotalDistance(Meters), this.actualAngle);
        this.actualPosition = new SwerveModulePosition(this.getTotalDistance(Meters), this.actualAngle);

        if (this.driveMotor != null) {
            
            double drive = (((this.getStateSpeed(MetersPerSecond) * this.driveGearRatio) / this.getWheelRadius(Meters)) / this.driveMotor.maxSpeed(RadiansPerSecond));
            this.driveMotor.set(Double.isNaN(drive) ? 0.0 : drive);
        }
        if (this.pivotMotor != null) {
            double pivot = this.pivotController.isPresent() ? 
                this.pivotController.get().calculate(this.getActualAngle().getRotations(), this.getStateAngle().getRotations())  // DO NOT CHANGE THIS TO RADIANS
                :
                this.getStateAngle().getRotations() - this.getActualAngle().getRotations();
            this.pivotMotor.set(Double.isNaN(pivot) ? 0.0 : pivot);
        }
    }

    /**
     * A configuration class used to make a SwerveModule.
     */
    public static class Builder {
        private Optional<SwerveMotor.SwerveMotorBuilder> driveMotor = Optional.empty(), pivotMotor = Optional.empty();
        private Optional<SwerveEncoder.SwerveEncoderBuilder> absoluteEncoder = Optional.empty();
        private Optional<PIDConstants> drivePID = Optional.empty(), pivotPID = Optional.empty();
        private Translation2d offset = new Translation2d();
        private double driveGearRatio = 1.0, pivotGearRatio = 1.0;
        private double wheelDiameter = 0.0;
        private boolean optimize = true;

        public Builder clone() {
            var dst = new Builder();
            if (this.drivePID != null) dst.drivePID = this.drivePID.map((pid) -> new PIDConstants(pid.kP, pid.kI, pid.kP));
            else dst.drivePID = null;
            if (this.pivotPID != null) dst.pivotPID = this.pivotPID.map((pid) -> new PIDConstants(pid.kP, pid.kI, pid.kP));
            else dst.pivotPID = null;
            dst.driveGearRatio = this.driveGearRatio;
            dst.pivotGearRatio = this.pivotGearRatio;
            dst.wheelDiameter = this.wheelDiameter;
            dst.optimize = this.optimize;
            return dst;
        }

        public SwerveModule build() {
            if (
                this.driveMotor.isEmpty() || 
                this.pivotMotor.isEmpty() ||
                this.absoluteEncoder.isEmpty()
            ) throw new IllegalStateException("Required fields missing when building SwerveModule!");
            return new SwerveModule(this);
        }

        public void fromJSON(JsonNode json) {
            if (json.has("driveMotor") && json.get("driveMotor").isObject()) {
                this.withDriveMotor(SwerveMotor.builderFromJSON(json.get("driveMotor")));
            }
            if (json.has("pivotMotor") && json.get("pivotMotor").isObject()) {
                this.withPivotMotor(SwerveMotor.builderFromJSON(json.get("pivotMotor")));
            }
            if (json.has("absoluteEncoder") && json.get("absoluteEncoder").isObject()) {
                this.withAbsoluteEncoder(SwerveEncoder.builderFromJSON(json.get("absoluteEncoder")));
            }
            if (json.has("drivePID") && json.get("drivePID").isObject()) {
                JsonNode json_inner = json.get("drivePID");
                if (json_inner.has("p") && json_inner.get("p").isDouble() && json_inner.has("i") && json_inner.get("i").isDouble() && json_inner.has("d") && json_inner.get("d").isDouble()) {
                    this.withDrivePID(new PIDConstants(json_inner.get("p").doubleValue(), json_inner.get("i").doubleValue(), json_inner.get("d").doubleValue()));
                }
            }
            if (json.has("pivotPID") && json.get("pivotPID").isObject()) {
                JsonNode json_inner = json.get("pivotPID");
                if (json_inner.has("p") && json_inner.get("p").isDouble() && json_inner.has("i") && json_inner.get("i").isDouble() && json_inner.has("d") && json_inner.get("d").isDouble()) {
                    this.withPivotPID(new PIDConstants(json_inner.get("p").doubleValue(), json_inner.get("i").doubleValue(), json_inner.get("d").doubleValue()));
                }
            }
            if (json.has("offset") && json.get("offset").isObject()) {
                JsonNode json_inner = json.get("offset");
                Distance unit = Meters;
                if (json_inner.has("unit") && json_inner.get("unit").isTextual()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.distanceFromName(json_inner.get("unit").textValue()), unit);
                }
                if (json_inner.has("x") && json_inner.get("x").isDouble() && json_inner.has("y") && json_inner.get("y").isDouble()) {
                    this.withOffset(new Translation2d(json_inner.get("x").doubleValue(), json_inner.get("y").doubleValue()), unit);
                }
            }

            if (json.has("wheelDiameter") && (json.get("wheelDiameter").isObject() || json.get("wheelDiameter").isDouble())) {
                JsonNode json_inner = json.get("wheelDiameter");
                Distance unit = Meters;
                if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.distanceFromName(json_inner.get("unit").textValue()), unit);
                    this.withWheelDiameter(json_inner.get("value").doubleValue(), unit);
                } else {
                    this.withWheelDiameter(json_inner.doubleValue(), unit);
                }
                
            }

            if (json.has("driveGearRatio") && json.get("driveGearRatio").isDouble()) {
                this.withDriveGearRatio(json.get("driveGearRatio").doubleValue());
            }

            if (json.has("pivotGearRatio") && json.get("pivotGearRatio").isDouble()) {
                this.withPivotGearRatio(json.get("pivotGearRatio").doubleValue());
            }

            if (json.has("optimize") && json.get("optimize").isBoolean()) {
                this.withOptimized(json.get("optimize").booleanValue());
            }
        }

        public Builder withDriveMotor(SwerveMotor.SwerveMotorBuilder motor) {
            this.driveMotor = Optional.of(motor);
            return this;
        }

        public Builder withPivotMotor(SwerveMotor.SwerveMotorBuilder motor) {
            this.pivotMotor = Optional.of(motor);
            return this;
        }

        public Builder withAbsoluteEncoder(SwerveEncoder.SwerveEncoderBuilder encoder) {
            this.absoluteEncoder = Optional.of(encoder);
            return this;
        }

        public Builder withOffset(double x, double y, Distance unit) {
            this.offset = new Translation2d(unit.toBaseUnits(x), unit.toBaseUnits(y));
            return this;
        }

        public Builder withOffset(Translation2d position, Distance unit) {
            return this.withOffset(position.getX(), position.getY(), unit);
        }

        public Builder withWheelDiameter(double diameter, Distance unit) {
            this.wheelDiameter = unit.toBaseUnits(diameter);
            return this;
        }

        public Builder withWheelDiameter(Measure<Distance> diameter) {
            this.wheelDiameter = diameter.baseUnitMagnitude();
            return this;
        }

        public Builder withDriveGearRatio(double ratio) {
            this.driveGearRatio = ratio;
            return this;
        }

        public Builder withPivotGearRatio(double ratio) {
            this.pivotGearRatio = ratio;
            return this;
        }

        public Builder withDrivePID(PIDConstants pid) {
            this.drivePID = Optional.of(pid);
            return this;
        }

        public Builder withPivotPID(PIDConstants pid) {
            this.pivotPID = Optional.of(pid);
            return this;
        }

        public Builder withOptimized(boolean enable) {
            this.optimize = enable;
            return this;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("stateSpeed", () -> this.getStateSpeed(MetersPerSecond) * 100, null);
        builder.addDoubleProperty("stateAngle", () -> this.getStateAngle(Degrees), null);
        builder.addDoubleProperty("actualSpeed", () -> this.getMeasuredVelocity(MetersPerSecond) * 100, null);
        builder.addDoubleProperty("actualAngle", () -> this.getActualAngle(Degrees), null);
        builder.addDoubleProperty("magnetOffset", () -> this.absoluteEncoder.getOffset(Degrees), (double val) -> this.absoluteEncoder.setOffset(val, Degrees));
        builder.addDoubleProperty("pControllerOutput", () -> {
            double driveDifference = this.getStateSpeed(MetersPerSecond)-this.getMeasuredVelocity(MetersPerSecond); 
            double driveSpeed = Math.abs(driveDifference) > 0.01 ? this.getMeasuredVelocity(MetersPerSecond)+Math.copySign(0.01, driveDifference) : this.getStateSpeed(MetersPerSecond);
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
