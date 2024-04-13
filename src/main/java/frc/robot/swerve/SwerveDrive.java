package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.OptionalDouble;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
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
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class SwerveDrive implements Sendable {
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private ChassisSpeeds speeds;
    private SwerveModule[] modules;
    private SwerveIMU imu;
    private Pose2d pose;
    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
    private Optional<Rotation2d> targetHeading = Optional.empty();
    private Field2d field = new Field2d();
    private boolean poseOverriden = false;
    private MutableMeasure<Velocity<Distance>> maxDriveSpeed = MutableMeasure.zero(MetersPerSecond);
    private MutableMeasure<Velocity<Angle>> maxTurnSpeed = MutableMeasure.zero(RadiansPerSecond);
    private MutableMeasure<Velocity<Velocity<Distance>>> maxDriveAccel = MutableMeasure.zero(MetersPerSecondPerSecond);
    private MutableMeasure<Velocity<Velocity<Angle>>> maxTurnAccel = MutableMeasure.zero(RadiansPerSecond.per(Second));
    private PIDController moduleDriveController = new PIDController(0, 0, 0), modulePivotController = new PIDController(0, 0, 0);
    private HolonomicDriveController driveController = new HolonomicDriveController(new PIDController(0, 0, 0), new PIDController(0, 0, 0), new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)));
    private boolean fieldCentric;
    private ChassisSpeeds actualSpeeds = new ChassisSpeeds();
    private Twist2d actualTwist = new Twist2d();
    private SwerveModuleState[] wheelStates;
    private SwerveModuleState[] actualWheelStates;
    private SwerveModulePosition[] wheelPositions;
    private SwerveModulePosition[] wheelDeltaPositions;

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
            positions[i] = this.modules[i].getModulePosition();
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

    public SwerveModule getSwerveModule(int id) {
        return this.modules[id];
    }

    public int getNumSwerveModules() {
        return this.modules.length;
    }

    public SwerveIMU getIMU() {
        return this.imu;
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    public void setMaxDriveSpeed(double speed, Velocity<Distance> unit) {
        this.maxDriveSpeed.mut_replace(speed, unit);
    }
    
    public void setMaxTurnSpeed(double speed, Velocity<Angle> unit) {
        this.maxTurnSpeed.mut_replace(speed, unit);
    }

    public void setMaxDriveAccel(double accel, Velocity<Velocity<Distance>> unit) {
        this.maxDriveAccel.mut_replace(accel, unit);
    }
    
    public void setMaxTurnAccel(double accel, Velocity<Velocity<Angle>> unit) {
        this.maxTurnAccel.mut_replace(accel, unit);
    }

    public double getMaxDriveSpeed(Velocity<Distance> unit) {
        return this.maxDriveSpeed.in(unit);
    }

    public double getMaxTurnSpeed(Velocity<Angle> unit) {
        return this.maxTurnSpeed.in(unit);
    }

    public double getMaxDriveAccel(Velocity<Velocity<Distance>> unit) {
        return this.maxDriveAccel.in(unit);
    }

    public double getMaxTurnAccel(Velocity<Velocity<Angle>> unit) {
        return this.maxTurnAccel.in(unit);
    }

    public double getPoseX(Distance unit) {
        return unit.convertFrom(this.pose.getX(), Meters);
    }

    public double getPoseY(Distance unit) {
        return unit.convertFrom(this.pose.getY(), Meters);
    }

    public Translation2d getPoseTranslationMeters() {
        return this.pose.getTranslation();
    }

    public double getPoseAngle(Angle unit) {
        return unit.convertFrom(this.pose.getRotation().getRadians(), Radians);
    }

    public Rotation2d getPoseAngle() {
        return this.pose.getRotation();
    }

    public OptionalDouble getTargetHeading(Angle unit) {
        return this.targetHeading.isPresent() ? OptionalDouble.of(unit.convertFrom(this.targetHeading.get().getRadians(), Radians)) : OptionalDouble.empty();
    }

    public Optional<Rotation2d> getTargetHeading() {
        return this.targetHeading;
    }

    

    public void drive(double x, double y, double h, HeadingControlMode mode, Optional<Boolean> fieldCentric) {
        x = MathUtil.clamp(x, -1.0, 1.0);
        y = MathUtil.clamp(y, -1.0, 1.0);
        h = MathUtil.clamp(h, -1.0, 1.0);

        this.targetSpeeds.vxMetersPerSecond = this.maxDriveSpeed.in(MetersPerSecond) * x;
        this.targetSpeeds.vyMetersPerSecond = this.maxDriveSpeed.in(MetersPerSecond) * y; 

        if (fieldCentric.isPresent()) {
            this.fieldCentric = fieldCentric.get();
        }

        switch (mode) {
            case kHeadingChange:
                // this.targetRSpeed.mut_replace(this.maxTurnSpeed.times(h));
                Rotation2d targetHeading = this.targetHeading.orElse(this.pose.getRotation());
                this.targetHeading = Optional.of(new Rotation2d(targetHeading.getRadians() + (this.maxTurnSpeed.in(RadiansPerSecond) * h * Robot.kDefaultPeriod)));
                break;
            case kHeadingSet:
                this.targetHeading = Optional.of(Rotation2d.fromRotations(h));
                break;
            case kSpeedOnly:
                this.targetSpeeds.omegaRadiansPerSecond = this.maxTurnSpeed.in(RadiansPerSecond) * h;
                this.targetHeading = Optional.empty();
            default:
                break;
            
        }


    }

    public void drive(ChassisSpeeds speeds, boolean relative) {
        if (relative) speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, this.pose.getRotation());
        this.targetSpeeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -this.maxDriveSpeed.in(MetersPerSecond), this.maxDriveSpeed.in(MetersPerSecond));
        this.targetSpeeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -this.maxDriveSpeed.in(MetersPerSecond), this.maxDriveSpeed.in(MetersPerSecond));
        this.targetSpeeds.omegaRadiansPerSecond = MathUtil.clamp(speeds.omegaRadiansPerSecond, -this.maxTurnSpeed.in(RadiansPerSecond), this.maxTurnSpeed.in(RadiansPerSecond));
        this.targetHeading = Optional.empty();
    } 

    public void setGlobalDrivePID(PIDConstants constants) {
        this.moduleDriveController = new PIDController(constants.kP, constants.kI, constants.kD);
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setDrivePID(constants); 
        }
    }

    public void setGlobalPivotPID(PIDConstants constants) {
        this.modulePivotController = new PIDController(constants.kP, constants.kI, constants.kD);
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setPivotPID(constants); 
        }
    }

    public void setDriveController(PIDConstants xyController, PIDConstants turnController) {
        this.driveController.getXController().setP(xyController.kP);
        this.driveController.getXController().setI(xyController.kI);
        this.driveController.getXController().setD(xyController.kD);
        this.driveController.getYController().setP(xyController.kP);
        this.driveController.getYController().setI(xyController.kI);
        this.driveController.getYController().setD(xyController.kD);
        this.driveController.getThetaController().setP(turnController.kP);
        this.driveController.getThetaController().setI(turnController.kI);
        this.driveController.getThetaController().setD(turnController.kD);
        this.driveController.getThetaController().setConstraints(new Constraints(this.maxTurnSpeed, this.maxTurnAccel));
        this.driveController.setEnabled(false);
    }

    public PIDController getGlobalDrivePID() {
        return this.moduleDriveController;
    }

    public PIDController getGlobalPivotPID() {
        return this.modulePivotController;
    }

    public HolonomicDriveController getDriveController() {
        return this.driveController;
    }



    public void zeroHeading() {
        this.imu.zeroHeading();
        this.targetHeading = this.targetHeading.map((x) -> new Rotation2d());
        this.overridePosition(new Pose2d(new Translation2d(this.pose.getX(), this.pose.getY()), new Rotation2d()));
    }

    public void overridePosition(Pose2d pose, boolean zeroIMU) {
        this.pose = pose;
        if (zeroIMU) this.imu.setHeadingOffset(pose.getRotation().getRadians(), Radians);
        this.poseOverriden = true;
    }

    public void overridePosition(Pose2d pose) {
        this.overridePosition(pose, false);
    }

    public Translation2d getDriveVelocityVector(Velocity<Distance> unit) {
       return new Translation2d(unit.convertFrom(this.actualSpeeds.vxMetersPerSecond, MetersPerSecond), unit.convertFrom(this.actualSpeeds.vyMetersPerSecond, MetersPerSecond));
    }

    public double getXVelocity(Velocity<Distance> unit) {
        return unit.convertFrom(this.actualSpeeds.vxMetersPerSecond, MetersPerSecond);
    }

    public double getYVelocity(Velocity<Distance> unit) {
        return unit.convertFrom(this.actualSpeeds.vyMetersPerSecond, MetersPerSecond);
    }

    public double getVelocity(Velocity<Distance> unit) {
        return Math.hypot(this.getXVelocity(unit), this.getYVelocity(unit));
    }

    public double getTurnVelocity(Velocity<Angle> unit) {
        return unit.convertFrom(this.actualSpeeds.omegaRadiansPerSecond, RadiansPerSecond);
    }

    public Pose2d getPose() {
        return this.pose;
    } 

    public ChassisSpeeds getActualSpeeds() {
        return this.actualSpeeds;
    }

    public void update() {

        if (this.speeds == null) {
            this.speeds = new ChassisSpeeds();
            System.out.println("Pose was null");
        }
        this.speeds = new ChassisSpeeds(
            limitAccelAndSpeed(this.targetXSpeed.in(MetersPerSecond), this.speeds.vxMetersPerSecond, Robot.kDefaultPeriod, this.maxDriveSpeed.in(MetersPerSecond), this.maxDriveAccel.in(MetersPerSecondPerSecond)),
            limitAccelAndSpeed(this.targetYSpeed.in(MetersPerSecond), this.speeds.vyMetersPerSecond, Robot.kDefaultPeriod, this.maxDriveSpeed.in(MetersPerSecond), this.maxDriveAccel.in(MetersPerSecondPerSecond)), 
            limitAccelAndSpeed((this.headingControlEnabled && !DriverStation.isAutonomousEnabled() ? 
                this.driveController.getThetaController().calculate(this.targetHeading.in(Radians) - this.pose.getRotation().getRadians(), 0.0)
                :
                this.targetRSpeed.in(RadiansPerSecond)
            ), this.speeds.omegaRadiansPerSecond, Robot.kDefaultPeriod, this.maxTurnSpeed.in(RadiansPerSecond), this.maxTurnAccel.in(RadiansPerSecond.per(Second)))
        );

        if (this.fieldCentric) {
            this.wheelStates = this.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(this.speeds, this.pose.getRotation()));
        } else {
            this.wheelStates = this.kinematics.toSwerveModuleStates(this.speeds);
        }

        

        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setState(this.wheelStates[i]);
            this.modules[i].update();
            this.wheelPositions[i] = this.modules[i].getActualPosition();
            this.wheelDeltaPositions[i] = this.modules[i].getActualDeltaPosition();
            // this.wheelPositions[i].angle = this.wheelPositions[i].angle.rotateBy(new Rotation2d(Math.PI));
            this.actualWheelStates[i] = this.modules[i].getActualState();
        }

        this.actualSpeeds = this.kinematics.toChassisSpeeds(this.actualWheelStates);

        this.actualTwist = this.kinematics.toTwist2d(this.wheelDeltaPositions);

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
        builder.addDoubleProperty("imu/headingSetpoint", () -> -this.targetHeading.in(Degrees), null);
        builder.addDoubleProperty("imu/magneticHeading", () -> this.imu.getCompassHeading(Degrees), null);
        builder.addDoubleProperty("imu/headingOffset", () -> -this.imu.getHeadingOffset(Degrees), (double offset) -> this.imu.setHeadingOffset(-offset, Degrees));
        builder.addDoubleProperty("imu/velocity/x", () -> this.getDriveVelocityVector(MetersPerSecond).getX(), null);
        builder.addDoubleProperty("imu/velocity/y", () -> this.getDriveVelocityVector(MetersPerSecond).getY(), null);
        builder.addDoubleProperty("imu/velocity/norm", () -> this.getDriveVelocityVector(MetersPerSecond).getNorm(), null);
        builder.addDoubleProperty("imu/velocity/angle", () -> this.getDriveVelocityVector(MetersPerSecond).getAngle().getDegrees(), null);
        builder.addDoubleArrayProperty("modules/actualStates", () -> {
            double[] vals = new double[this.actualWheelStates.length << 1];
            for (int i = 0; i < this.actualWheelStates.length; i++) {
                vals[(i << 1)] = this.actualWheelStates[i].angle.getDegrees();
                vals[(i << 1) + 1] = this.actualWheelStates[i].speedMetersPerSecond;
            }
            return vals;
        }, null);
        
    }

    private static double limitAccelAndSpeed(double value, double oldValue, double period, double maxSpeed, double maxAccel) {
        var maxChange = maxAccel * period;
        var tmp = MathUtil.clamp(value, oldValue-maxChange, oldValue+maxChange);
        var result =  MathUtil.clamp(tmp, -maxSpeed, maxSpeed);
        return result;
    }

    public static class SwerveDriveConfig {

    }
}
