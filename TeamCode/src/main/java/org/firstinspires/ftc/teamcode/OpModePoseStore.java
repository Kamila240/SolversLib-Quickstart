package org.firstinspires.ftc.teamcode;
import com.pedropathing.geometry.Pose;

public final class OpModePoseStore {
    private OpModePoseStore() {}

    private static Pose savedPose = null;

    public static void save(Pose pose) {
        if (pose == null) {
            return;
        }
        savedPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static boolean hasSavedPose() {
        return savedPose != null;
    }

    public static Pose getSavedPoseOr(Pose fallback) {
        if (savedPose != null) {
            return new Pose(savedPose.getX(), savedPose.getY(), savedPose.getHeading());
        }
        return fallback;
    }

    public static void clear() {
        savedPose = null;
    }
}