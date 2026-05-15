package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class cs {

    public static double poseX = 9.12414123412521;
    public static double poseY = 8.66121495327102;
    public static double poseHeading = 90;
    public static double stopper_closed = 0.5;
    public static double stopper_open = 0.7;
    public static double turret_center = 0.5;
    public static double turret_range = 270.0;
    public static  double RED_GOAL_X = 138;
    public static  double RED_GOAL_Y = 138;
    public static  double BLUE_GOAL_X = 6;
    public static  double BLUE_GOAL_Y = 138;

    public static final String FRONT_LEFT = "fl";
    public static final String FRONT_RIGHT = "fr";
    public static final String REAR_LEFT = "rl";
    public static final String REAR_RIGHT = "rr";

    // --- Mechanism Motor Names ---
    // intake 1 and intake2 are not inverted they have same direction
    public static final String INTAKE_1 = "intake1";
    public static final String INTAKE_2 = "intake2";
    public static final String FLYWHEEL_LEFT = "shooter2";
    public static final String FLYWHEEL_RIGHT = "shooter1";

    // --- Servo Names ---
    // --- turret1 and turret2 not inverted,but hood1 and hood2 yes
    public static final String TURRET_1 = "turret1";
    public static final String TURRET_2 = "turret2";
    public static final String HOOD = "hood";
    public static final String STOPPER = "stopper";
    public static Pose getStartPose() {
        return new Pose(
                poseX,
                poseY,
                Math.toRadians(poseHeading)
        );
    }
}