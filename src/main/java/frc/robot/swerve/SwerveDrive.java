package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrive {
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private ChassisSpeeds speeds;
    private SwerveModule[] modules;
    private SwerveIMU imu;

    public SwerveDrive(Pose2d initialPose, SwerveIMU imu, SwerveModule... modules) {
        this.imu = imu;
        this.modules = modules;
        Translation2d[] positions = new Translation2d[this.modules.length];
        for (int i = 0; i < this.modules.length; i++) {
            positions[i] = this.modules[i].getModulePosition();
        } 
        this.kinematics = new SwerveDriveKinematics(positions);
        this.odometry = new SwerveDriveOdometry(
            this.kinematics, 
            this.imu.getHeading(), 
            new SwerveModulePosition[] {
                
            }, initialPose)
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

    }
}
