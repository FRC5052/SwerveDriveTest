package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive implements Sendable {
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private ChassisSpeeds speeds;
    private SwerveModule[] modules;
    private SwerveIMU imu;
    private Pose2d pose;
    private Rotation2d headingSetpoint;
    private Field2d field;
    private boolean poseOverriden;

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
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[this.modules.length];
        for (int i = 0; i < this.modules.length; i++) {
            positions[i] = this.modules[i].getModulePosition();
            modulePositions[i] = new SwerveModulePosition(0, this.modules[i].getActualAngle());
            SmartDashboard.putData("swerveDrive/modules/" + i, this.modules[i]);
        } 
        this.kinematics = new SwerveDriveKinematics(positions);
        this.odometry = new SwerveDriveOdometry(
            this.kinematics, 
            this.imu.getHeading(), 
            modulePositions, 
            this.pose
        );
        this.headingSetpoint = new Rotation2d();
        this.field = new Field2d();
        this.field.setRobotPose(this.pose);
        SmartDashboard.putData("field", this.field);
        SmartDashboard.putData("swerveDrive", this);
    }
    
    public void setSpeeds(double x, double y, Rotation2d h, boolean fieldRel, HeadingControlMode mode) {
        switch (mode) {
            case kHeadingChange:
                this.headingSetpoint = this.headingSetpoint.plus(h);
                break;
            case kHeadingSet:
                this.headingSetpoint = new Rotation2d(h.getRadians());
                break;
            default:
                break;
            
        }

        if (mode.requiresControlLoop()) {
            var diff = this.imu.getHeading().unaryMinus().minus(this.headingSetpoint);
            h = Math.abs(diff.getRotations()) > 0.5 ? Rotation2d.fromRotations(Math.copySign(0.5, diff.getRotations())) : diff;
        }

        if (fieldRel) {
            this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, h.getRadians(), this.imu.getHeading().unaryMinus());
        } else {
            this.speeds = new ChassisSpeeds(x, y, h.getRadians());
        }
    }

    public void zeroHeading() {
        this.imu.zeroHeading();
        this.headingSetpoint = new Rotation2d();
    }

    public void overridePosition(Pose2d pose) {
        this.pose = pose;
        this.poseOverriden = true;
    }

    public void update() {
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(this.speeds);
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[this.modules.length];
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setState(states[i]);
            modulePositions[i] = new SwerveModulePosition(this.modules[i].getTotalDistance(), this.modules[i].getActualAngle());
            this.modules[i].update();
        } 
        if (this.poseOverriden) {
            this.odometry.resetPosition(
                this.imu.getHeading(), 
                modulePositions, 
                this.pose
            );
        } else {
            this.pose = this.odometry.update(
                this.imu.getHeading(), 
                modulePositions    
            );
        }
        this.field.setRobotPose(this.pose);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("imu/heading", () -> this.imu.getHeading().getDegrees(), null);
        builder.addDoubleProperty("imu/rawHeading", () -> this.imu.getRawHeading().getDegrees(), null);
        builder.addDoubleProperty("imu/headingSetpoint", () -> this.headingSetpoint.getDegrees(), null);
        builder.addDoubleProperty("imu/magneticHeading", () -> this.imu.getCompassHeading().getDegrees(), null);
        builder.addDoubleProperty("imu/headingOffset", () -> this.imu.getHeadingOffset().getDegrees(), (double offset) -> this.imu.setHeadingOffset(Rotation2d.fromDegrees(offset)));
        builder.addDoubleArrayProperty("imu/acceleration", () -> new double[] {
            this.imu.getWorldAccel().getX(), 
            this.imu.getWorldAccel().getY(), 
            this.imu.getWorldAccel().getZ(), 
        }, null);
    }

    
}
