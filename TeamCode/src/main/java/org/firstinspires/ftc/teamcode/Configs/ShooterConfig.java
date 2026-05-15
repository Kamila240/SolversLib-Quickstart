package org.firstinspires.ftc.teamcode.Configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    public static double TARGET_VELOCITY = 3000.0;
    public static double READY_TOLERANCE = 50.0;
    public static double REFEED_THRESHOLD = 0.95;

    public static double kV = 0.00030;
    public static double kS = 0.03;
    public static double kP = 0.00020;

    public static double INTAKE_FEED_POWER = 1.0;
    public static double INTAKE_MANUAL_POWER = 1.0;

    public static double STOPPER_OPEN = 0.60;
    public static double STOPPER_CLOSED = 0.20;

    public static boolean USE_VOLTAGE_COMP = true;
    public static boolean REVERSE_LEFT_FLYWHEEL = true;
    public static boolean REVERSE_RIGHT_FLYWHEEL = false;
}