package frc.robot.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;

public interface SwerveIMU {
    public Rotation3d getRotation();
    public Rotation2d getHeading();
    public Rotation2d getRawHeading();
    public Rotation2d getCompassHeading();
    public Translation3d getWorldAccel();
    public void calibrate();
    public void zeroHeading();
    public Rotation2d getHeadingOffset();
    public void setHeadingOffset(Rotation2d offset);
    // public void syncHeadingToMagnet();
    
    public class NavXSwerveIMU implements SwerveIMU {
        private AHRS navX;

        public NavXSwerveIMU() {
            this.navX = new AHRS();
        }

        @Override
        public Rotation3d getRotation() {
            return new Rotation3d(new Quaternion(
                this.navX.getQuaternionW(), 
                this.navX.getQuaternionX(), 
                this.navX.getQuaternionY(), 
                this.navX.getQuaternionZ()
            ));
        }

        @Override
        public Rotation2d getHeading() {
            return Rotation2d.fromDegrees((double)this.navX.getFusedHeading()-this.navX.getAngleAdjustment());
        }

        @Override
        public Rotation2d getRawHeading() {
            return this.getHeading().plus(this.getHeadingOffset());
        }

        @Override
        public Rotation2d getCompassHeading() {
            return Rotation2d.fromDegrees((double)this.navX.getCompassHeading());
        }

        @Override
        public Translation3d getWorldAccel() {
            return new Translation3d(
                this.navX.getWorldLinearAccelX(), 
                this.navX.getWorldLinearAccelY(), 
                this.navX.getWorldLinearAccelZ()
            );
        }

        @Override
        public void calibrate() {
            
        }

        @Override
        public void zeroHeading() {
            this.setHeadingOffset(this.getRawHeading());
        }

        @Override
        public Rotation2d getHeadingOffset() {
            return Rotation2d.fromDegrees(this.navX.getAngleAdjustment());
        }

        @Override
        public void setHeadingOffset(Rotation2d offset) {
            this.navX.setAngleAdjustment(offset.getDegrees());
        }        
    }
}
