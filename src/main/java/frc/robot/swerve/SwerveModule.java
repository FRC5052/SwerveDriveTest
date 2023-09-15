package frc.robot.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private SwerveModuleConfig cfg;
    private SwerveModuleState state;
    
    public SwerveModule(SwerveModuleConfig cfg) {
        this.cfg = cfg;
        this.state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    public void setState(SwerveModuleState state) {
        this.state = state;
    }

    public SwerveModuleState getState() {
        return this.state;
    }

    public Translation2d getModulePosition() {
        return this.cfg.position;
    }

    public Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(this.cfg.absoluteEncoder.getAbsolutePosition()-180);
    }

    public void periodic() {
        this.cfg.driveMotor.set(this.state.speedMetersPerSecond*this.cfg.driveGearRatio);
        this.cfg.driveMotor.getEncoder().getInverted();
    }

    public class SwerveModuleConfig {
        public SwerveMotor driveMotor, pivotMotor;
        public SwerveEncoder absoluteEncoder;
        public PIDController driveController, pivotController;
        public Rotation2d offset;
        public Translation2d position;
        public double driveGearRatio, pivotGearRatio;
        public double wheelDiameter;
    }
}
