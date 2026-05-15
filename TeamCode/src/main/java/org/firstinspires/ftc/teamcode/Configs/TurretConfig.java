package org.firstinspires.ftc.teamcode.Configs;
import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConfig {

    public static double SERVO_CENTER = 0.5;
    public static double SERVO_MIN = 0.0;
    public static double SERVO_MAX = 1.0;
    public static double TUR_OFFSETS_LEFT = 25.0;
    public static double TUR_OFFSET_RIGHT = 15.0; //-10,0 //0
    public static double TUR_OFFSET_3RD = -20.0;
    public static double TUR_OFFSET_4TH = -70.0;
    public static double TUR_OFFSET_5TH = -10.0;

    // -180 -> 0.0
    //  0   -> 0.5
    // +180 -> 1.0
    public static double SERVO_RANGE_DEG = 360.0;

    public static double MANUAL_STEP_DEG = 2.5; //
    public static double MAX_STEP_DEG_PER_LOOP = 3.0;
    public static double STICK_DEADBAND = 0.05;

    public static boolean REVERSE_SERVO1 = false;
    public static boolean REVERSE_SERVO2 = false;
}