package org.firstinspires.ftc.teamcode.Configs;
import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConfig_goal {

    public static double SERVO_CENTER = 0.52;
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public static double GEAR_RATIO = 0.75;
    public static double OFFSET = 0;

    //public static double TUR_OFFSETS_LEFT = 10;
    //public static double TUR_OFFSET_RIGHT = 15; //-10,0 //0
    //public static double TUR_OFFSET_3RD = -25;
    //public static double TUR_OFFSET_4TH = 0;
    //public static double TUR_OFFSET_5TH = -27;

    // -180 -> 0.0
    //  0   -> 0.5
    // +180 -> 1.0
    public static double SERVO_RANGE_DEG = 346.0;

    public static double MANUAL_STEP_DEG = 2.5;
    public static double MAX_STEP_DEG_PER_LOOP = 4.7;
    public static double STICK_DEADBAND = 0.05;

    public static boolean REVERSE_SERVO1 = false;
    public static boolean REVERSE_SERVO2 = false;
}