package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.swerve.SwerveModule;

public class SwerveDrive implements Sendable {
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final SwerveModule[] modules;
    private final SwerveIMU imu;
    private Pose2d pose;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
    private Optional<Rotation2d> targetHeading = Optional.empty();
    private Field2d field = new Field2d();
    private boolean poseOverriden = false;
    private double maxDriveSpeed = 0.0;
    private double maxTurnSpeed = 0.0;
    private double maxDriveAccel = 0.0;
    private double maxTurnAccel = 0.0;
    private PIDController moduleDriveController = new PIDController(0, 0, 0), modulePivotController = new PIDController(0, 0, 0);
    private ProfiledPIDController headingController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    private boolean fieldCentric;
    private ChassisSpeeds actualSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] wheelStates;
    private final SwerveModuleState[] actualWheelStates;
    private final SwerveModulePosition[] wheelPositions;
    private final SwerveModulePosition[] wheelDeltaPositions;

    // public static final Struct<SwerveDrive> struct = new Struct<SwerveDrive>() {
        
    // };

    public enum HeadingControlMode {
        // Value is to be interpreted as raw speed.
        kSpeedOnly,
        // Value is the new heading setpoint.
        kHeadingSet,
        // Value is the change in the heading.
        kHeadingChange,
        ;

        public boolean requiresControlLoop() {
            switch (this) {
                case kHeadingChange:
                case kHeadingSet:
                    return true;
                default:
                    return false;
                
            }
        }
    }
    
    /**
     * Constructs a new SwerveDrive object with the given initial pose, imu, and modules.
     * @param initialPose The Pose2d object representing the robot's starting position.
     * @param imu A SwerveIMU representing the robot's onboard IMU.
     * @param modules A list of SwerveModules representing the robot's swerve drive modules.
     */
    public SwerveDrive(Pose2d initialPose, SwerveIMU imu, SwerveModule... modules) {
        this.imu = imu;
        this.imu.calibrate();
        this.modules = modules;
        this.pose = initialPose;
        Translation2d[] positions = new Translation2d[this.modules.length];

        this.wheelPositions = new SwerveModulePosition[this.modules.length];
        this.wheelDeltaPositions = new SwerveModulePosition[this.modules.length];
        this.actualWheelStates = new SwerveModuleState[this.modules.length];
        for (int i = 0; i < this.modules.length; i++) {
            positions[i] = this.modules[i].getModuleOffset();
            this.wheelPositions[i] = new SwerveModulePosition(0, Rotation2d.fromRadians(this.modules[i].getActualAngle(Radians)));
            this.actualWheelStates[i] = new SwerveModuleState();
            SmartDashboard.putData("swerveDrive/modules/module" + i, this.modules[i]);
        } 
        this.kinematics = new SwerveDriveKinematics(positions);
        this.odometry = new SwerveDriveOdometry(
            this.kinematics, 
            new Rotation2d(-this.imu.getHeading(Radians)), 
            this.wheelPositions, 
            this.pose
        );
        this.field.setRobotPose(this.pose);
        this.imu.zeroHeading();
        SmartDashboard.putData("field", this.field);
        SmartDashboard.putData("swerveDrive/drivePID", this.moduleDriveController);
        SmartDashboard.putData("swerveDrive/pivotPID", this.modulePivotController);
        SmartDashboard.putData("swerveDrive", this);
    }

    /**
     * Returns one of the modules in this swerve drive, based on the given index.
     * @param index The index to retrieve the module from.
     * @return The SwerveModule at the given index.
     */
    public SwerveModule getSwerveModule(int index) {
        return this.modules[index];
    }

    /**
     * Returns the number of moduless in this swerve drive.
     * @return The number of modules.
     */
    public int getNumSwerveModules() {
        return this.modules.length;
    }

    /**
     * Returns this swerve drive's IMU.
     * @return This swerve drive's SwerveIMU.
     */
    public SwerveIMU getIMU() {
        return this.imu;
    }

    /**
     * Configures whether this swerve drive should be field centric or not.
     * @param fieldCentric If this swerve drive should be field centric.
     */
    public void setIsFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    /**
     * Returns whether this swerve drive is field centric or not.
     * @return If this swerve drive is field centric.
     */
    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    /**
     * Configures this swerve drive's maximum translational velocity.
     * @param speed The new maximum velocity of this swerve drive, in the given units.
     * @param unit The unit of velocity to use to convert the new value.
     */
    public void setMaxDriveSpeed(double speed, Velocity<Distance> unit) {
        this.maxDriveSpeed = unit.toBaseUnits(speed);
    }

    /**
     * Configures this swerve drive's maximum translational velocity.
     * @param speed The new maximum velocity of this swerve drive.
     */
    public void setMaxDriveSpeed(Measure<Velocity<Distance>> speed) {
        this.maxDriveSpeed = speed.baseUnitMagnitude();
    }
    
    /**
     * Configures this swerve drive's maximum rotational velocity.
     * @param speed The new maximum angular velocity of this swerve drive, in the given units.
     * @param unit The unit of angular velocity to use to convert the new value.
     */
    public void setMaxTurnSpeed(double speed, Velocity<Angle> unit) {
        this.maxTurnSpeed = unit.toBaseUnits(speed);
    }

    /**
     * Configures this swerve drive's maximum rotational velocity.
     * @param speed The new maximum angular velocity of this swerve drive.
     */
    public void setMaxTurnSpeed(Measure<Velocity<Angle>> speed) {
        this.maxTurnSpeed = speed.baseUnitMagnitude();
    }

    /**
     * Configures this swerve drive's maximum translational acceleration.
     * @param accel The new maximum acceleration of this swerve drive, in the given units.
     * @param unit The unit of acceleration to use to convert the new value.
     */
    public void setMaxDriveAccel(double accel, Velocity<Velocity<Distance>> unit) {
        this.maxDriveAccel = unit.toBaseUnits(accel);
    }

    /**
     * Configures this swerve drive's maximum translational acceleration.
     * @param accel The new maximum acceleration of this swerve drive.
     */
    public void setMaxDriveAccel(Measure<Velocity<Velocity<Distance>>> accel) {
        this.maxDriveAccel = accel.baseUnitMagnitude();
    }
    
    /**
     * Configures this swerve drive's maximum rotational acceleration.
     * @param accel The new maximum angular acceleration of this swerve drive, in the given units.
     * @param unit The unit of angular acceleration to use to convert the new value.
     */
    public void setMaxTurnAccel(double accel, Velocity<Velocity<Angle>> unit) {
        this.maxTurnAccel = unit.toBaseUnits(accel);
    }

    /**
     * Configures this swerve drive's maximum rotational acceleration.
     * @param accel The new maximum angular acceleration of this swerve drive.
     */
    public void setMaxTurnAccel(Measure<Velocity<Velocity<Angle>>> accel) {
        this.maxTurnAccel = accel.baseUnitMagnitude();
    }

    /**
     * Returns this swerve drive's maximum translational velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The maximum velocity of this swerve drive, in the given units.
     */
    public double getMaxDriveSpeed(Velocity<Distance> unit) {
        return unit.fromBaseUnits(this.maxDriveSpeed);
    }

    /**
     * Returns this swerve drive's maximum rotational velocity.
     * @param unit The unit of angular velocity to use to convert the value.
     * @return The maximum angular velocity of this swerve drive, in the given units.
     */
    public double getMaxTurnSpeed(Velocity<Angle> unit) {
        return unit.fromBaseUnits(this.maxTurnSpeed);
    }


    /**
     * Returns this swerve drive's maximum translational acceleration.
     * @param unit The unit of acceleration to use to convert the value.
     * @return The maximum acceleration of this swerve drive, in the given units.
     */
    public double getMaxDriveAccel(Velocity<Velocity<Distance>> unit) {
        return unit.fromBaseUnits(this.maxDriveAccel);
    }

    /**
     * Returns this swerve drive's maximum rotational acceleration.
     * @param unit The unit of angular acceleration to use to convert the value.
     * @return The maximum angular acceleration of this swerve drive, in the given units.
     */
    public double getMaxTurnAccel(Velocity<Velocity<Angle>> unit) {
        return unit.fromBaseUnits(this.maxTurnAccel);
    }

    /**
     * Returns the X component of this swerve drive's current pose.
     * @param unit The unit of distance to use to convert the value.
     * @return The X position of this swerve drive on the field, in the given units.
     */
    public double getPoseX(Distance unit) {
        return unit.convertFrom(this.pose.getX(), Meters);
    }

    /**
     * Returns the Y component of this swerve drive's current pose.
     * @param unit The unit of distance to use to convert the value.
     * @return The Y position of this swerve drive on the field, in the given units.
     */
    public double getPoseY(Distance unit) {
        return unit.convertFrom(this.pose.getY(), Meters);
    }

    /**
     * Returns the translational component of this swerve drive's current pose.
     * @return A Translation2d representing this swerve drive's field position.
     */
    public Translation2d getPosePositionMeters() {
        return this.pose.getTranslation();
    }

    /**
     * Returns the theta (angular) component of this swerve drive's current pose.
     * @param unit The unit of angle to use to convert the value.
     * @return The heading of this swerve drive on the field, in the given units.
     */
    public double getPoseAngle(Angle unit) {
        return unit.convertFrom(this.pose.getRotation().getRadians(), Radians);
    }

    /**
     * Returns the theta (angular) component of this swerve drive's current pose.
     * @return The heading of this swerve drive on the field.
     */
    public Rotation2d getPoseAngle() {
        return this.pose.getRotation();
    }

    /** 
     * Returns the target heading of this swerve drive, if present.
     * @param unit The unit of angle to use to convert the value.
     * @return The target heading value, in the given units, if present.
     */
    public OptionalDouble getTargetHeading(Angle unit) {
        return this.targetHeading.isPresent() ? OptionalDouble.of(unit.convertFrom(this.targetHeading.get().getRadians(), Radians)) : OptionalDouble.empty();
    }

    /** 
     * Returns the target heading of this swerve drive, if present.
     * @return The target heading value, if present.
     */
    public Optional<Rotation2d> getTargetHeading() {
        return this.targetHeading;
    }

    /** 
     * Returns the actual, measured heading of this swerve drive, as reported by the IMU.
     * @param unit The unit of angle to use to convert the value.
     * @return The measured heading value, in the given units.
     */
    public double getActualHeading(Angle unit) {
        return this.imu.getHeading(unit);
    }

    /**
     * Sets the targeted drive speed to the given values.
     * @param x The desired X speed. The value should be normalized (-1 to 1).
     * @param y The desired Y speed. The value should be normalized (-1 to 1).
     * @param h The desired angular speed or heading, depending on the mode. The value should be normalized (-1 to 1).
     * @param mode The heading control mode to use. This will change how the h value is interpreted.
     * @param fieldCentric If present, overrides the current fieldCentric setting.
     */
    public void drive(double x, double y, double h, HeadingControlMode mode, Optional<Boolean> fieldCentric) {
        x = MathUtil.clamp(x, -1.0, 1.0);
        y = MathUtil.clamp(y, -1.0, 1.0);
        h = MathUtil.clamp(h, -1.0, 1.0);

        this.targetSpeeds.vxMetersPerSecond = this.maxDriveSpeed * x;
        this.targetSpeeds.vyMetersPerSecond = this.maxDriveSpeed * y; 

        if (fieldCentric.isPresent()) {
            this.fieldCentric = fieldCentric.get();
        }

        switch (mode) {
            case kHeadingChange:
                // this.targetRSpeed.mut_replace(this.maxTurnSpeed.times(h));
                Rotation2d targetHeading = this.targetHeading.orElse(this.pose.getRotation());
                this.targetHeading = Optional.of(new Rotation2d(targetHeading.getRadians() + (this.maxTurnSpeed * h * Robot.kDefaultPeriod)));
                break;
            case kHeadingSet:
                this.targetHeading = Optional.of(Rotation2d.fromRotations(h));
                break;
            case kSpeedOnly:
                this.targetSpeeds.omegaRadiansPerSecond = this.maxTurnSpeed * h;
                this.targetHeading = Optional.empty();
            default:
                break;
            
        }


    }

    /**
     * Sets the targeted drive speed to the given values.
     * @param speeds The ChassisSpeeds object to apply. The values 
     * @param isRobotRelative Whether the speeds value is robot centric, otherwise it is assumed to be field centric.
     */
    public void drive(ChassisSpeeds speeds, boolean isRobotRelative) {
        if (isRobotRelative) speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, this.pose.getRotation());
        this.targetSpeeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -this.maxDriveSpeed, this.maxDriveSpeed);
        this.targetSpeeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -this.maxDriveSpeed, this.maxDriveSpeed);
        this.targetSpeeds.omegaRadiansPerSecond = MathUtil.clamp(speeds.omegaRadiansPerSecond, -this.maxTurnSpeed, this.maxTurnSpeed);
        this.targetHeading = Optional.empty();
    } 

    /**
     * Configures the drive PID constants for all swerve modules.
     * @param constants The constants to apply.
     */
    public void setModuleDrivePID(PIDConstants constants) {
        this.moduleDriveController = new PIDController(constants.kP, constants.kI, constants.kD);
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setDrivePID(Optional.of(constants)); 
        }
    }

    /**
     * Configures the pivot PID constants for all swerve modules.
     * @param constants The constants to apply.
     */
    public void setModulePivotPID(PIDConstants constants) {
        this.modulePivotController = new PIDController(constants.kP, constants.kI, constants.kD);
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setPivotPID(Optional.of(constants)); 
        }
    }

    /**
     * Configures the heading PID constants for heading targeting.
     * @param constants The constants to apply.
     */
    public void setHeadingPID(PIDConstants constants) {
        this.headingController.setP(constants.kP);
        this.headingController.setI(constants.kI);
        this.headingController.setD(constants.kD);
        this.headingController.setConstraints(new Constraints(this.maxTurnSpeed, this.maxTurnAccel));
    }

    /**
     * Returns the drive PID controller for all swerve modules.
     * @return The common drive PIDController.
     */
    public PIDController getModuleDrivePID() {
        return this.moduleDriveController;
    }

    /**
     * Returns the pivot PID controller for all swerve modules.
     * @return The common pivot PIDController.
     */
    public PIDController getModulePivotPID() {
        return this.modulePivotController;
    }

    /**
     * Returns the heading PID controller for heading targeting.
     * @return The common heading ProfiledPIDController.
     */
    public ProfiledPIDController getHeadingController() {
        return this.headingController;
    }

    /**
     * Resets the heading of this swerve drive such that the new reported heading is zero.
     */
    public void zeroHeading() {
        this.imu.zeroHeading();
        this.targetHeading = this.targetHeading.map((x) -> new Rotation2d());
        this.setPose(new Pose2d(new Translation2d(this.pose.getX(), this.pose.getY()), new Rotation2d()));
    }

    /** 
     * Sets the pose of this swerve drive.
     * @param pose The new pose to apply.
     */
    public void setPose(Pose2d pose) {
        this.pose = pose;
        this.poseOverriden = true;
    }

    /**
     * Returns the pose of this swerve drive.
     * @return The pose of this swerve drive, in meters.
     */
    public Pose2d getPose() {
        return this.pose;
    }

    /**
     * Returns the X component of this swerve drive's velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The X component of this swerve drive's velocity, in the given units.
     */
    public double getVelocityX(Velocity<Distance> unit) {
        return unit.convertFrom(this.actualSpeeds.vxMetersPerSecond, MetersPerSecond);
    }

    /**
     * Returns the Y component of this swerve drive's velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The Y component of this swerve drive's velocity, in the given units.
     */
    public double getVelocityY(Velocity<Distance> unit) {
        return unit.convertFrom(this.actualSpeeds.vyMetersPerSecond, MetersPerSecond);
    }

    /**
     * Returns the magnitude of this swerve drive's velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The magnitude of this swerve drive's velocity, in the given units.
     */
    public double getVelocityMagnitude(Velocity<Distance> unit) {
        return Math.hypot(this.getVelocityX(unit), this.getVelocityY(unit));
    }

    /**
     * Returns the direction of this swerve drive's velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The direction of this swerve drive's velocity, in the given units.
     */
    public double getVelocityDirection(Angle unit) {
        return unit.convertFrom(Math.atan2(this.getVelocityY(MetersPerSecond), this.getVelocityX(MetersPerSecond)), Radians);
    }

    /**
     * Returns the theta component of this swerve drive's velocity.
     * @param unit The unit of angular velocity to use to convert the value.
     * @return The theta component of this swerve drive's velocity, in the given units.
     */
    public double getTurnVelocity(Velocity<Angle> unit) {
        return unit.convertFrom(this.actualSpeeds.omegaRadiansPerSecond, RadiansPerSecond);
    } 

    /**
     * Returns the velocity of this swerve drive.
     * @return A ChassisSpeeds representing this swerve drive's velocity.
     */
    public ChassisSpeeds getActualSpeeds() {
        return this.actualSpeeds;
    }

    /**
     * Returns the field of this swerve drive.
     * @return A Field2d representing this swerve drive's field.
     */
    public Field2d getField() {
        return this.field;
    }

    /**
     * Updates the internal logic of this swerve drive. This should be called in a command or subsystem's periodic function.
     */
    public void update() {
        this.speeds = new ChassisSpeeds(
            SwerveUtil.limitAccelAndSpeed(this.targetSpeeds.vxMetersPerSecond, this.speeds.vxMetersPerSecond, Robot.kDefaultPeriod, this.maxDriveSpeed, this.maxDriveAccel),
            SwerveUtil.limitAccelAndSpeed(this.targetSpeeds.vyMetersPerSecond, this.speeds.vyMetersPerSecond, Robot.kDefaultPeriod, this.maxDriveSpeed, this.maxDriveAccel), 
            SwerveUtil.limitAccelAndSpeed((this.targetHeading.isPresent() && !DriverStation.isAutonomousEnabled() ? 
                this.headingController.calculate(this.targetHeading.get().getRadians() - this.pose.getRotation().getRadians(), 0.0)
                :
                this.targetSpeeds.omegaRadiansPerSecond
            ), this.speeds.omegaRadiansPerSecond, Robot.kDefaultPeriod, this.maxTurnSpeed, this.maxTurnAccel)
        );

        if (this.fieldCentric) {
            this.wheelStates = this.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(this.speeds, this.pose.getRotation()));
        } else {
            this.wheelStates = this.kinematics.toSwerveModuleStates(this.speeds);
        }

        

        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setTargetState(this.wheelStates[i]);
            this.modules[i].update();
            this.wheelPositions[i] = this.modules[i].getActualPosition();
            this.wheelDeltaPositions[i] = this.modules[i].getActualDeltaPosition();
            // this.wheelPositions[i].angle = this.wheelPositions[i].angle.rotateBy(new Rotation2d(Math.PI));
            this.actualWheelStates[i] = this.modules[i].getActualState();
        }

        this.actualSpeeds = this.kinematics.toChassisSpeeds(this.actualWheelStates);

        if (this.poseOverriden) {
            this.odometry.resetPosition(
                new Rotation2d(this.imu.getHeading(Radians)), 
                this.wheelPositions, 
                this.pose
            );
            this.poseOverriden = false;
        } else {
            this.pose = this.odometry.update(
                new Rotation2d(this.imu.getHeading(Radians)), 
                this.wheelPositions    
            );
        }
        this.field.setRobotPose(this.pose);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("imu/heading", () -> this.getPoseAngle(Degrees), null);
        builder.addDoubleProperty("imu/rawHeading", () -> -this.imu.getRawHeading(Degrees), null);
        builder.addDoubleProperty("imu/headingSetpoint", () -> -this.targetHeading.orElse(new Rotation2d()).getDegrees(), null);
        builder.addDoubleProperty("imu/magneticHeading", () -> this.imu.getCompassHeading(Degrees), null);
        builder.addDoubleProperty("imu/headingOffset", () -> -this.imu.getHeadingOffset(Degrees), (double offset) -> this.imu.setHeadingOffset(-offset, Degrees));
        builder.addDoubleProperty("imu/velocity/x", () -> this.getVelocityX(MetersPerSecond), null);
        builder.addDoubleProperty("imu/velocity/y", () -> this.getVelocityY(MetersPerSecond), null);
        builder.addDoubleProperty("imu/velocity/norm", () -> this.getVelocityMagnitude(MetersPerSecond), null);
        builder.addDoubleProperty("imu/velocity/angle", () -> this.getVelocityDirection(Degrees), null);
        builder.addDoubleArrayProperty("modules/actualStates", () -> {
            double[] vals = new double[this.actualWheelStates.length << 1];
            for (int i = 0; i < this.actualWheelStates.length; i++) {
                vals[(i << 1)] = this.actualWheelStates[i].angle.getDegrees();
                vals[(i << 1) + 1] = this.actualWheelStates[i].speedMetersPerSecond;
            }
            return vals;
        }, null);
        
    }

    public static class Builder {
        private Optional<SwerveModule.Builder[]> modules = Optional.empty();
        private Optional<SwerveIMU.SwerveIMUBuilder> imu = Optional.empty();
        private Pose2d initialPose = new Pose2d();
        private Optional<PIDConstants> moduleDrivePID = Optional.empty();
        private Optional<PIDConstants> modulePivotPID = Optional.empty();
        private Optional<PIDConstants> headingPID = Optional.empty();
        private OptionalDouble maxDriveSpeed = OptionalDouble.empty();
        private OptionalDouble maxTurnSpeed = OptionalDouble.empty();
        private OptionalDouble maxDriveAccel = OptionalDouble.empty();
        private OptionalDouble maxTurnAccel = OptionalDouble.empty();
        private boolean fieldCentric = false;

        public Builder withModules(SwerveModule.Builder... modules) {
            this.modules = Optional.of(modules);
            return this;
        }

        public Builder withIMU(SwerveIMU.SwerveIMUBuilder imu) {
            this.imu = Optional.of(imu);
            return this;
        }

        public Builder withInitialPose(Pose2d pose) {
            this.initialPose = pose;
            return this;
        }

        public Builder withModuleDrivePID(PIDConstants constants) {
            this.moduleDrivePID = Optional.of(constants);
            return this;
        }

        public Builder withModulePivotPID(PIDConstants constants) {
            this.modulePivotPID = Optional.of(constants);
            return this;
        }

        public Builder withHeadingPID(PIDConstants constants) {
            this.headingPID = Optional.of(constants);
            return this;
        }

        public Builder withMaxDriveSpeed(double speed, Velocity<Distance> unit) {
            this.maxDriveSpeed = OptionalDouble.of(unit.toBaseUnits(speed));
            return this;
        }

        public Builder withMaxTurnSpeed(double speed, Velocity<Angle> unit) {
            this.maxTurnSpeed = OptionalDouble.of(unit.toBaseUnits(speed));
            return this;
        }

        public Builder withMaxDriveAccel(double speed, Velocity<Velocity<Distance>> unit) {
            this.maxDriveAccel = OptionalDouble.of(unit.toBaseUnits(speed));
            return this;
        }

        public Builder withMaxTurnAccel(double speed, Velocity<Velocity<Angle>> unit) {
            this.maxTurnAccel = OptionalDouble.of(unit.toBaseUnits(speed));
            return this;
        }

        public Builder withFieldCentric(boolean fieldCentric) {
            this.fieldCentric = fieldCentric;
            return this;
        }

        public void fromJSON(JsonNode json) {
            if (json.has("modules") && json.get("modules").isArray()) {
                JsonNode modulesJSON = json.get("modules");
                SwerveModule.Builder[] modules = new SwerveModule.Builder[modulesJSON.size()];
                for (int i = 0; i < modules.length; i++) {
                    if (modulesJSON.has(i) && modulesJSON.get(i).isObject()) {
                        modules[i] = new SwerveModule.Builder();
                        modules[i].fromJSON(modulesJSON.get(i));
                    }
                }
                this.withModules(modules);
            }
            if (json.has("imu") && json.get("imu").isObject()) {
                this.withIMU(SwerveIMU.builderFromJSON(json.get("imu")));
            }
            
            if (json.has("headingPID") && json.get("headingPID").isObject()) {
                JsonNode json_inner = json.get("headingPID");
                if (json_inner.has("p") && json_inner.get("p").isDouble() && json_inner.has("i") && json_inner.get("i").isDouble() && json_inner.has("d") && json_inner.get("d").isDouble()) {
                    this.withHeadingPID(new PIDConstants(json_inner.get("p").doubleValue(), json_inner.get("i").doubleValue(), json_inner.get("d").doubleValue()));
                }
            }

            if (json.has("maxDriveSpeed") && (json.get("maxDriveSpeed").isObject() || json.get("maxDriveSpeed").isDouble())) {
                JsonNode json_inner = json.get("maxDriveSpeed");
                Velocity<Distance> unit = MetersPerSecond;
                if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.velocityFromName(json_inner.get("unit").textValue()), unit);
                    this.withMaxDriveSpeed(json_inner.get("value").doubleValue(), unit);
                } else {
                    this.withMaxDriveSpeed(json_inner.doubleValue(), unit);
                }
            }

            if (json.has("maxTurnSpeed") && (json.get("maxTurnSpeed").isObject() || json.get("maxTurnSpeed").isDouble())) {
                JsonNode json_inner = json.get("maxTurnSpeed");
                Velocity<Angle> unit = RadiansPerSecond;
                if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.angularVelocityFromName(json_inner.get("unit").textValue()), unit);
                    this.withMaxTurnSpeed(json_inner.get("value").doubleValue(), unit);
                } else {
                    this.withMaxTurnSpeed(json_inner.doubleValue(), unit);
                }
            }

            if (json.has("maxDriveAccel") && (json.get("maxDriveAccel").isObject() || json.get("maxDriveAccel").isDouble())) {
                JsonNode json_inner = json.get("maxDriveAccel");
                Velocity<Velocity<Distance>> unit = MetersPerSecondPerSecond;
                if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.accelFromName(json_inner.get("unit").textValue()), unit);
                    this.withMaxDriveAccel(json_inner.get("value").doubleValue(), unit);
                } else {
                    this.withMaxDriveAccel(json_inner.doubleValue(), unit);
                }
            }

            if (json.has("maxTurnAccel") && (json.get("maxTurnAccel").isObject() || json.get("maxTurnAccel").isDouble())) {
                JsonNode json_inner = json.get("maxTurnAccel");
                Velocity<Velocity<Angle>> unit = RadiansPerSecond.per(Second);
                if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.angularAccelFromName(json_inner.get("unit").textValue()), unit);
                    this.withMaxTurnAccel(json_inner.get("value").doubleValue(), unit);
                } else {
                    this.withMaxTurnAccel(json_inner.doubleValue(), unit);
                }
            }

            if (json.has("fieldCentric") && json.get("fieldCentric").isBoolean()) {
                this.withFieldCentric(json.get("fieldCentric").booleanValue());
            }
        }

        public SwerveDrive build() {
            if (this.modules.isEmpty()) {
                throw new IllegalStateException("Modules field was empty");
            }
            if (this.imu.isEmpty()) {
                throw new IllegalStateException("IMU field was empty");
            }
            SwerveModule.Builder[] modules = this.modules.get();
            SwerveModule[] new_modules = new SwerveModule[modules.length];
            for (int i = 0; i < new_modules.length; i++) {
                new_modules[i] = modules[i].build();
            }
            var tmp = new SwerveDrive(this.initialPose, this.imu.get().build(), new_modules);
            this.apply(tmp);
            return tmp;
        }

        public void apply(SwerveDrive swerveDrive) {
            this.moduleDrivePID.ifPresent((PIDConstants constants) -> swerveDrive.setModuleDrivePID(constants));
            this.modulePivotPID.ifPresent((PIDConstants constants) -> swerveDrive.setModulePivotPID(constants));
            this.headingPID.ifPresent((PIDConstants constants) -> swerveDrive.setHeadingPID(constants));
            this.maxDriveSpeed.ifPresent((double speed) -> swerveDrive.setMaxDriveSpeed(speed, MetersPerSecond));
            this.maxTurnSpeed.ifPresent((double speed) -> swerveDrive.setMaxTurnSpeed(speed, RadiansPerSecond));
            this.maxDriveAccel.ifPresent((double accel) -> swerveDrive.setMaxDriveAccel(accel, MetersPerSecondPerSecond));
            this.maxTurnAccel.ifPresent((double accel) -> swerveDrive.setMaxTurnAccel(accel, RadiansPerSecond.per(Second)));
            swerveDrive.setIsFieldCentric(this.fieldCentric);
        }
    }
}
