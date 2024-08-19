package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

public final class SwerveUtil {
    static double limitAccelAndSpeed(double value, double oldValue, double period, double maxSpeed, double maxAccel) {
        var maxChange = maxAccel * period;
        var tmp = MathUtil.clamp(value, oldValue-maxChange, oldValue+maxChange);
        var result =  MathUtil.clamp(tmp, -maxSpeed, maxSpeed);
        return result;
    }

    static Distance distanceFromName(String name) {
        switch (name.toLowerCase()) {
            case "m":
                return Meters;
            case "cm":
                return Centimeters;
            case "mm":
                return Millimeters;
            case "in":
                return Inches;
            case "ft":
                return Feet;
            default:
                return null;
        }
    }

    static Velocity<Distance> velocityFromName(String name) {
        switch (name.toLowerCase()) {
            case "m/s":
                return MetersPerSecond;
            case "in/s":
                return InchesPerSecond;
            case "ft/s":
                return FeetPerSecond;
            default:
                return null;
        }
    }

    static Velocity<Velocity<Distance>> accelFromName(String name) {
        switch (name.toLowerCase()) {
            case "m/s2":
                return MetersPerSecondPerSecond;
            case "in/s2":
                return InchesPerSecond.per(Second);
            case "ft/s2":
                return FeetPerSecond.per(Second);
            case "g":
                return Gs;
            default:
                return null;
        }
    }

    static Angle angleFromName(String name) {
        switch (name.toLowerCase()) {
            case "rad":
                return Radians;
            case "deg":
                return Degrees;
            case "rot":
            case "rev":
                return Rotations;
            default:
                return null;
        }
    }

    static Velocity<Angle> angularVelocityFromName(String name) {
        switch (name.toLowerCase()) {
            case "rad/s":
                return RadiansPerSecond;
            case "deg/s":
                return DegreesPerSecond;
            case "rot/s":
            case "rev/s":
                return RotationsPerSecond;
            case "rpm":
                return RPM;
            default:
                return null;
        }
    }

    static Velocity<Velocity<Angle>> angularAccelFromName(String name) {
        switch (name.toLowerCase()) {
            case "rad/s2":
                return RadiansPerSecond.per(Second);
            case "deg/s2":
                return DegreesPerSecond.per(Second);
            case "rot/s2":
            case "rev/s2":
                return RotationsPerSecond.per(Second);
            default:
                return null;
        }
    }

    static Time timeFromName(String name) {
        switch (name.toLowerCase()) {
            case "s":
                return Seconds;
            case "ms":
                return Milliseconds;
            case "us":
                return Microseconds;
            case "m":
                return Minutes;
            default:
                return null;
        }
    }
}
