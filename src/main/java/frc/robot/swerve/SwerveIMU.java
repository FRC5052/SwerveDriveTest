package frc.robot.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Gs;
import static edu.wpi.first.units.Units.Radians;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

public abstract class SwerveIMU {

    /** 
     * Returns the 3D rotation of this IMU.
     * @return The Rotation3d object representing the 3D rotation this IMU is reporting.
     */
    public abstract Rotation3d getRotation();

    /** 
     * Returns the heading of this IMU in the given unit, with heading correction applied. 
     * @param unit
     */
    public abstract double getHeading(Angle unit);

    /** Returns the heading of this IMU in the given unit, without heading correction. */
    public abstract double getRawHeading(Angle unit);

    /** Returns the heading of this IMU relative to the planet's magnetic field (if present), in the given unit. 
     * 
     * Note that on some implementations, the IMU has to be calibrated manually for this function to work correctly.
     */
    public abstract double getCompassHeading(Angle unit);
    public abstract Translation3d getWorldAccel(Velocity<Velocity<Distance>> unit);
    public abstract void calibrate();
    public abstract void zeroHeading();
    public abstract double getHeadingOffset(Angle unit);
    public abstract void setHeadingOffset(double offset, Angle unit);
    // public void syncHeadingToMagnet();
    
    public class NavXSwerveIMU extends SwerveIMU {
        private AHRS navX;

        public NavXSwerveIMU() {
            this.navX = new AHRS();
        }

        public NavXSwerveIMU(SPI.Port port) {
            this.navX = new AHRS(port);
        }

        public NavXSwerveIMU(I2C.Port port) {
            this.navX = new AHRS(port);
        }

        public NavXSwerveIMU(SerialPort.Port port) {
            this.navX = new AHRS(port);
        }

        @Override
        public Rotation3d getRotation() {
            return new Rotation3d(new Quaternion(
                (double)this.navX.getQuaternionW(), 
                (double)this.navX.getQuaternionX(), 
                (double)this.navX.getQuaternionY(), 
                (double)this.navX.getQuaternionZ()
            ));
        }

        @Override
        public double getHeading(Angle unit) {
            return this.getRawHeading(unit) - this.getHeadingOffset(unit);
        }

        @Override
        public double getRawHeading(Angle unit) {
            return unit.convertFrom(-(double)this.navX.getFusedHeading(), Degrees);
        }

        @Override
        public double getCompassHeading(Angle unit) {
            return unit.convertFrom(-(double)this.navX.getCompassHeading(), Degrees);
        }

        @Override
        public Translation3d getWorldAccel(Velocity<Velocity<Distance>> unit) {
            return new Translation3d(
                unit.convertFrom(this.navX.getWorldLinearAccelX(), Gs), 
                unit.convertFrom(this.navX.getWorldLinearAccelY(), Gs), 
                unit.convertFrom(this.navX.getWorldLinearAccelZ(), Gs)
            );
        }

        @Override
        public void calibrate() {
            
        }

        @Override
        public void zeroHeading() {
            this.setHeadingOffset(this.getRawHeading(Radians), Radians);
        }

        @Override
        public double getHeadingOffset(Angle unit) {
            return unit.convertFrom(-this.navX.getAngleAdjustment(), Degrees);
        }

        @Override
        public void setHeadingOffset(double offset, Angle unit) {
            this.navX.setAngleAdjustment(-Degrees.convertFrom(offset, unit));
        }        
    }
}
