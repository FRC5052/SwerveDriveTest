package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Robot;

public class SwerveDrive implements Sendable {
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private ChassisSpeeds speeds;
    private SwerveModule[] modules;
    private SwerveIMU imu;
    private Pose2d pose;
    private Rotation2d headingSetpoint;
    private Translation2d oldPosition;
    private Translation2d positionSetpoint;
    private Translation2d currentVelocity;
    private Field2d field;
    private boolean poseOverriden;
    private double maxDriveSpeed, maxTurnSpeed;
    private PIDController globalDriveController, globalPivotController, turnController;
    private boolean fieldCentric;

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
        this.globalDriveController = new PIDController(0, 0, 0);
        this.globalPivotController = new PIDController(0, 0, 0);
        this.turnController = new PIDController(0, 0, 0);
        SmartDashboard.putData("field", this.field);
        SmartDashboard.putData("swerveDrive/drivePID", this.globalDriveController);
        SmartDashboard.putData("swerveDrive/pivotPID", this.globalPivotController);
        SmartDashboard.putData("swerveDrive/turnPID", this.turnController);
        SmartDashboard.putData("swerveDrive", this);
        this.imu.zeroHeading();
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    public void setMaxDriveSpeed(double speed) {
        this.maxDriveSpeed = speed;
    }
    
    public void setMaxTurnSpeed(double speed) {
        this.maxTurnSpeed = speed;
    }

    public double getMaxDriveSpeed() {
        return this.maxDriveSpeed;
    }

    public double getMaxTurnSpeed() {
        return this.maxTurnSpeed;
    }

    public void setSpeeds(double x, double y, Rotation2d h, HeadingControlMode mode) {

        if (this.positionSetpoint != null) {
            x = querp(this.oldPosition.getX(), this.positionSetpoint.getX(), this.pose.getX()); 
            y = querp(this.oldPosition.getY(), this.positionSetpoint.getY(), this.pose.getY());
            h = new Rotation2d();
            System.out.println(x + " " + y);
            mode = HeadingControlMode.kSpeedOnly;
            if (Math.abs(x) <= 0.05 && Math.abs(y) <= 0.05) {
                this.cancelMove();
            }
            
        } else {
            h = h.unaryMinus().times(maxTurnSpeed / 2);
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
                h = Rotation2d.fromRotations(this.turnController.calculate(this.imu.getHeading().minus(this.headingSetpoint).getRotations(), 0.0)).unaryMinus();
            }
        }
        
        x = MathUtil.clamp(x, -1.0, 1.0);
        y = MathUtil.clamp(y, -1.0, 1.0);
        h = Rotation2d.fromRotations(MathUtil.clamp(h.getRotations(), -1.0, 1.0));

        if (this.fieldCentric) {
            this.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x * maxDriveSpeed, y * maxDriveSpeed, h.times(maxTurnSpeed).getRadians(), this.imu.getHeading().unaryMinus());
        } else {
            this.speeds = new ChassisSpeeds(x * maxDriveSpeed, y * maxDriveSpeed, h.times(maxTurnSpeed).getRadians());
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

    public void moveTo(Translation2d pos) {
        this.positionSetpoint = new Translation2d(pos.getX(), pos.getY());
        this.oldPosition = new Translation2d(this.pose.getX(), this.pose.getY());
    }

    public void moveBy(Translation2d off) {
        this.moveTo(new Translation2d(this.pose.getX()+off.getX(), this.pose.getY()+off.getY()));
        System.out.println(this.positionSetpoint);
        System.out.println(this.oldPosition);
        System.out.println(this.pose);
    }

    public void cancelMove() {
        System.out.println("Stopped");
        this.positionSetpoint = null;
        this.oldPosition = null;
    }

    public Translation2d getVelocityVector() {
        if (this.currentVelocity != null) return this.currentVelocity;
        Translation2d velocity = new Translation2d();
        for (SwerveModule module : this.modules) {
            velocity = velocity.plus(module.getVelocityVector());
        }
        return velocity.div((double)this.modules.length).rotateBy(this.imu.getHeading());
    }

    private double querp(double start, double end, double x) {

        x -= start;
        end -= start;
        double a = Math.copySign(1.0, end) * x;
        if (end == 0 || a < 0 || a > Math.abs(end)) return 0.0; 
        else return -Math.copySign(1.0, end) * (((4 / Math.pow(end, 2)) * x * (x - end) * 0.95) - 0.05);
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
        this.currentVelocity = null;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("imu/heading", () -> this.imu.getHeading().getDegrees(), null);
        builder.addDoubleProperty("imu/rawHeading", () -> this.imu.getRawHeading().getDegrees(), null);
        builder.addDoubleProperty("imu/headingSetpoint", () -> this.headingSetpoint.getDegrees(), null);
        builder.addDoubleProperty("imu/magneticHeading", () -> this.imu.getCompassHeading().getDegrees(), null);
        builder.addDoubleProperty("imu/headingOffset", () -> this.imu.getHeadingOffset().getDegrees(), (double offset) -> this.imu.setHeadingOffset(Rotation2d.fromDegrees(offset)));
        builder.addDoubleProperty("imu/velocity/x", () -> this.getVelocityVector().getX(), null);
        builder.addDoubleProperty("imu/velocity/y", () -> this.getVelocityVector().getY(), null);
        builder.addDoubleProperty("imu/velocity/norm", () -> this.getVelocityVector().getNorm(), null);
        builder.addDoubleProperty("imu/velocity/angle", () -> this.getVelocityVector().getAngle().getDegrees(), null);
    }

    
}
