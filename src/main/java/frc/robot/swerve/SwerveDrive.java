package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive implements Sendable {
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private ChassisSpeeds speeds;
    private SwerveModule[] modules;
    private SwerveIMU imu;
    private Pose2d pose;

    public SwerveDrive(Pose2d initialPose, SwerveIMU imu, SwerveModule... modules) {
        this.imu = imu;
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
        SmartDashboard.putData("swerveDrive", this);
    }
    
    public void setSpeeds(double x, double y, double h, boolean fieldRel) {
        if (fieldRel) {
            this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, h, this.imu.getHeading());
        } else { 
            this.speeds = new ChassisSpeeds(x, y, h);
        }
    }

    public void update() {
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(this.speeds);
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[this.modules.length];
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setState(states[i]);
            modulePositions[i] = new SwerveModulePosition(this.modules[i].getTotalDistance(), this.modules[i].getActualAngle());
            this.modules[i].update();
        } 
        this.pose = this.odometry.update(
            this.imu.getHeading(), 
            modulePositions    
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("swerveDrive/imu/heading", () -> this.imu.getHeading().getDegrees(), null);
        builder.addDoubleProperty("swerveDrive/imu/magneticHeading", () -> this.imu.getCompassHeading().getDegrees(), null);
    }

    
}
