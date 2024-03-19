package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTags {
    private static NetworkTable aprilTagTable;

    private static AprilTag cachedAprilTag = new AprilTag();
    private static boolean foundAprilTag = false;

    public static class AprilTag {
        public int id;
        public Pose3d pose;
        public Translation2d screenPosition;
    }

    public static void init() {
        if (aprilTagTable == null) aprilTagTable = NetworkTableInstance.getDefault().getTable("apriltag");
    }

    public static void setIdQuery(int id) {
        init();
        aprilTagTable.getEntry("idQuery").setInteger(id);
        aprilTagTable.getEntry("idQuery").clearPersistent();
    }



    public static void update() {
        init();
        if (aprilTagTable.getEntry("tagID").getInteger(0) == 0) foundAprilTag = false;
        else {
            cachedAprilTag.id = (int)aprilTagTable.getEntry("tagID").getInteger(0);
            cachedAprilTag.pose = new Pose3d(new Translation3d(
                aprilTagTable.getEntry("translation").getDoubleArray(new double[] {0.0, 0.0, 0.0})[0],
                aprilTagTable.getEntry("translation").getDoubleArray(new double[] {0.0, 0.0, 0.0})[1],
                aprilTagTable.getEntry("translation").getDoubleArray(new double[] {0.0, 0.0, 0.0})[2]
            ), new Rotation3d(
                aprilTagTable.getEntry("rotation").getDoubleArray(new double[] {0.0, 0.0, 0.0})[0],
                aprilTagTable.getEntry("rotation").getDoubleArray(new double[] {0.0, 0.0, 0.0})[1],
                aprilTagTable.getEntry("rotation").getDoubleArray(new double[] {0.0, 0.0, 0.0})[2]
            ));
            cachedAprilTag.screenPosition = new Translation2d(
                aprilTagTable.getEntry("center").getDoubleArray(new double[] {0.0, 0.0})[0],
                aprilTagTable.getEntry("center").getDoubleArray(new double[] {0.0, 0.0})[1]
            );
            foundAprilTag = true;
        }
    }

    public static AprilTag getAprilTag() {
        init();
        return foundAprilTag ? cachedAprilTag : null;
    }
}
