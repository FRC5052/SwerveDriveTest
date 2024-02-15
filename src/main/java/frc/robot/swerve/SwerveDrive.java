package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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
    private Rotation2d headingSetpoint = new Rotation2d();
    private Pose2d startPose = new Pose2d();
    private Pose2d endPose = new Pose2d();
    private Field2d field = new Field2d();
    private boolean poseOverriden = false;
    private MutableMeasure<Velocity<Distance>> maxDriveSpeed = MutableMeasure.zero(MetersPerSecond);
    private MutableMeasure<Velocity<Angle>> maxTurnSpeed = MutableMeasure.zero(RadiansPerSecond);
    private PIDController globalDriveController = new PIDController(0, 0, 0), globalPivotController = new PIDController(0, 0, 0), turnController = new PIDController(0, 0, 0);
    private boolean fieldCentric;
    private ChassisSpeeds actualChassisSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] wheelStates;
    private SwerveModuleState[] actualWheelStates;
    private SwerveModulePosition[] wheelPositions;

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

        boolean requiresControlLoop() {
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
            new Rotation2d(this.imu.getHeading(Radians)), 
            this.wheelPositions, 
            this.pose
        );
        this.field.setRobotPose(this.pose);
        this.imu.zeroHeading();
        SmartDashboard.putData("field", this.field);
        SmartDashboard.putData("swerveDrive/drivePID", this.globalDriveController);
        SmartDashboard.putData("swerveDrive/pivotPID", this.globalPivotController);
        SmartDashboard.putData("swerveDrive/turnPID", this.turnController);
        SmartDashboard.putData("swerveDrive", this);
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

    public double getMaxDriveSpeed(Velocity<Distance> unit) {
        return this.maxDriveSpeed.in(unit);
    }

    public double getMaxTurnSpeed(Velocity<Angle> unit) {
        return this.maxTurnSpeed.in(unit);
    }

    public void setSpeeds(double x, double y, Rotation2d h, HeadingControlMode mode) {
        if (this.endPose != null) {
            x = interpolate(this.startPose.getX(), this.endPose.getX(), this.pose.getX()); 
            y = interpolate(this.startPose.getY(), this.endPose.getY(), this.pose.getY());
            h = new Rotation2d(interpolate(
                MathUtil.angleModulus(this.startPose.getRotation().getRadians()), 
                MathUtil.angleModulus(this.endPose.getRotation().getRadians()), 
                MathUtil.angleModulus(this.pose.getRotation().getRadians())
            ));

            System.out.println(x + " " + y + " " + h.getRadians());

            mode = HeadingControlMode.kSpeedOnly;
            if (this.pose.getTranslation().getDistance(this.endPose.getTranslation()) < 0.001) {
                this.cancelMove();
            }
            
        } else {
            x = -x;
            y = -y;

            switch (mode) {
                case kHeadingChange:
                    this.headingSetpoint = this.headingSetpoint.plus(h.times(Robot.kDefaultPeriod));
                    break;
                case kHeadingSet:
                    this.headingSetpoint = new Rotation2d(h.getRadians());
                    break;
                default:
                    break;
                
            }

            if (mode.requiresControlLoop()) {
                h = Rotation2d.fromRotations(this.turnController.calculate(this.imu.getHeading(Rotations), this.headingSetpoint.getRotations())).unaryMinus();
            }
        }
        
        x = MathUtil.clamp(x, -1.0, 1.0);
        y = MathUtil.clamp(y, -1.0, 1.0);
        h = Rotation2d.fromRotations(MathUtil.clamp(h.getRotations(), -1.0, 1.0));

        if (this.fieldCentric) {
            this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x * this.maxDriveSpeed.in(MetersPerSecond), y * this.maxDriveSpeed.in(MetersPerSecond), h.getRadians() * this.maxTurnSpeed.in(RadiansPerSecond), new Rotation2d(this.imu.getHeading(Radians)).unaryMinus());
        } else {
            this.speeds = new ChassisSpeeds(x * this.maxDriveSpeed.in(MetersPerSecond), y * this.maxDriveSpeed.in(MetersPerSecond), h.getRadians() * this.maxTurnSpeed.in(RadiansPerSecond));
        }
    }

    public void setGlobalDrivePID(PIDController controller) {
        this.globalDriveController.setP(controller.getP());
        this.globalDriveController.setI(controller.getI());
        this.globalDriveController.setD(controller.getD());
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setDrivePID(this.globalDriveController, false); 
        }
    }

    public void setGlobalPivotPID(PIDController controller) {
        this.globalPivotController.setP(controller.getP());
        this.globalPivotController.setI(controller.getI());
        this.globalPivotController.setD(controller.getD());
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setPivotPID(this.globalPivotController, false); 
        }
    }

    public void setTurnPID(PIDController controller) {
        this.turnController.setP(controller.getP());
        this.turnController.setI(controller.getI());
        this.turnController.setD(controller.getD());
        this.turnController.enableContinuousInput(-0.5, 0.5);
    }

    public PIDController getGlobalDrivePID() {
        return this.globalDriveController;
    }

    public PIDController getGlobalPivotPID() {
        return this.globalPivotController;
    }

    public PIDController getTurnPID() {
        return this.turnController;
    }

    public void zeroHeading() {
        this.imu.zeroHeading();
        this.headingSetpoint = new Rotation2d();
        this.overridePosition(new Pose2d(new Translation2d(this.pose.getX(), this.pose.getY()), new Rotation2d()));
    }

    public void overridePosition(Pose2d pose) {
        this.pose = pose;
        this.poseOverriden = true;
    }

    public void poseTo(Pose2d newPose) {
        this.endPose = newPose;
        this.startPose = new Pose2d(this.pose.getX(), this.pose.getY(), new Rotation2d(this.pose.getRotation().getRadians()));
    }

    public void moveTo(Translation2d pos) {
        this.poseTo(new Pose2d(pos, new Rotation2d(this.pose.getRotation().getRadians())));
    }

    public void moveBy(Translation2d off) {
        this.moveTo(this.pose.getTranslation().plus(off));
    }

    public void rotateTo(Rotation2d heading) {
        this.poseTo(new Pose2d(new Translation2d(this.pose.getX(), this.pose.getY()), heading));
    }

    public void rotateBy(Rotation2d angle) {
        this.rotateTo(this.pose.getRotation().plus(angle));
    }

    public void cancelMove() {
        this.endPose = null;
        this.startPose = null;
    }

    public Translation2d getDriveVelocityVector(Velocity<Distance> unit) {
       return new Translation2d(unit.convertFrom(this.actualChassisSpeeds.vxMetersPerSecond, MetersPerSecond), unit.convertFrom(this.actualChassisSpeeds.vyMetersPerSecond, MetersPerSecond));
    }

    public double getXVelocity(Velocity<Distance> unit) {
        return unit.convertFrom(this.actualChassisSpeeds.vxMetersPerSecond, MetersPerSecond);
    }

    public double getYVelocity(Velocity<Distance> unit) {
        return unit.convertFrom(this.actualChassisSpeeds.vyMetersPerSecond, MetersPerSecond);
    }

    public double getTurnVelocity(Velocity<Angle> unit) {
        return unit.convertFrom(this.actualChassisSpeeds.omegaRadiansPerSecond, RadiansPerSecond);
    }

    private static double interpolate(double start, double end, double x) {
        if (Math.abs(x-start) < 0.0 || Math.abs(x-start) >= Math.abs(end-start)*1.25) return 0;
        return Math.copySign(MathUtil.clamp(1.0 - Math.pow((Math.abs(end-start) - Math.abs(x-start)), 2), -0.5, 1.0), end-start) / 2;
        // x -= start;
        // end -= start;
        // double a = Math.copySign(1.0, end) * x;
        // if (end == 0 || a < 0 || a > Math.abs(end)) return 0.0; 
        // else return (1.0 / (
        //     x - (end + Math.copySign(1.0, end))
        // )) + Math.copySign((
        //     1.0 /* + 1.0 / (Math.abs(end) + 1)*/
        // ), end);
    }

    public void update() {
        
        this.wheelStates = this.kinematics.toSwerveModuleStates(this.speeds);
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setState(this.wheelStates[i]);
            this.modules[i].update();
            this.wheelPositions[i] = new SwerveModulePosition(this.modules[i].getTotalDistance(Meters), this.modules[i].getActualAngle());
            this.actualWheelStates[i] = this.modules[i].getActualState();
        }

        this.actualChassisSpeeds = this.kinematics.toChassisSpeeds(this.actualWheelStates);

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
        System.out.println(this.pose);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("imu/heading", () -> this.imu.getHeading(Degrees), null);
        builder.addDoubleProperty("imu/rawHeading", () -> this.imu.getRawHeading(Degrees), null);
        builder.addDoubleProperty("imu/headingSetpoint", () -> this.headingSetpoint.getDegrees(), null);
        builder.addDoubleProperty("imu/magneticHeading", () -> this.imu.getCompassHeading(Degrees), null);
        builder.addDoubleProperty("imu/headingOffset", () -> this.imu.getHeadingOffset(Degrees), (double offset) -> this.imu.setHeadingOffset(Degrees, offset));
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

    
}
