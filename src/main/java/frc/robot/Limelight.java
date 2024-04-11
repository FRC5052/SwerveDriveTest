package frc.robot;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.*;

public class Limelight {
    private static NetworkTable limelightTable;

    private static void init() {
        if (limelightTable == null) {
            limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        }
    }

    public static boolean hasTarget() {
        init();
        return limelightTable.getEntry("tv").getInteger(0) != 0;
    }

    public static OptionalDouble getTargetX() {
        init();
        return hasTarget() ? OptionalDouble.of(limelightTable.getEntry("tx").getDouble(0.0)) : OptionalDouble.empty(); 
    }

    public static OptionalDouble getTargetY() {
        init();
        return hasTarget() ? OptionalDouble.of(limelightTable.getEntry("tx").getDouble(0.0)) : OptionalDouble.empty(); 
    }

    public static Optional<Pose2d> getFieldCentricRobotPose(Distance distanceUnit, boolean useMegaTag2) {
        init();
        if (hasTarget()) {
            double[] poseArray = useMegaTag2 ? limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]) : limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            return Optional.of(new Pose2d(distanceUnit.convertFrom(poseArray[0], Meters), distanceUnit.convertFrom(poseArray[1], Meters), new Rotation2d(Radians.convertFrom(poseArray[5], Degrees))));
        } else {
            return Optional.empty();
        }
    }

    public static void setRobotYaw(double angle, double angularVelocity, Angle angleUnit, Velocity<Angle> angularVelocityUnit) {
        init();
        limelightTable.getEntry("robot_orientation_set").setDoubleArray(new double[] {
            Degrees.convertFrom(angle, angleUnit),
            DegreesPerSecond.convertFrom(angularVelocity, angularVelocityUnit),
            0.0,
            0.0,
            0.0,
            0.0
        });
    }
}
