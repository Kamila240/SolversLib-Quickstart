package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShooterSubsystemAngleSS extends ShooterSubsystem {

    private final Servo angleLeft;
    private final Servo angleRight;

    private final Servo stopperServo;

    // Stopper positions
    public static double STOPPER_INITIAL_POS = 0.50;  // начальная / закрытая позиция
    public static double STOPPER_SHOOT_POS = 0.60;    // позиция при trigger / shooting

    // Одна оптимальная скорость для всех позиций
    public static double OPTIMAL_TARGET_VELOCITY = 1950.0;

    // Углы шутера
    public static double ANGLE_CLOSE = 0.0;  // нижняя стартовая позиция
    public static double ANGLE_FAR = 0.45;   // дальняя позиция

    public static double MIN_ANGLE_POS = 0.0;
    public static double MAX_ANGLE_POS = 1.0;

    public static boolean RIGHT_SERVO_REVERSED = true;

    // Если distance <= CLOSE_DISTANCE_MAX -> close angle
    // Если distance > CLOSE_DISTANCE_MAX -> far angle
    public static double CLOSE_DISTANCE_MAX = 60.0;

    private double activeAnglePosition = ANGLE_CLOSE;
    private double activeDistanceToGoal = 0.0;
    private String activeZone = "IDLE";

    public ShooterSubsystemAngleSS(HardwareMap hardwareMap) {
        super(hardwareMap);

        angleLeft = hardwareMap.servo.get("AngleSpeedLeft");
        angleRight = hardwareMap.servo.get("AngleSpeedRight");

        stopperServo = hardwareMap.servo.get("stopper");

        // При INIT сразу ставим:
        // угол вниз + stopper в начальную позицию
        setCloseAngle();
        setStopperInitial();
    }

    public void setShooterAngle(double position) {
        double safePos = clampAngle(position);

        activeAnglePosition = safePos;

        angleLeft.setPosition(safePos);

        if (RIGHT_SERVO_REVERSED) {
            angleRight.setPosition(1.0 - safePos);
        } else {
            angleRight.setPosition(safePos);
        }
    }

    public void setCloseAngle() {
        activeZone = "CLOSE";
        setShooterAngle(ANGLE_CLOSE);
    }

    public void setFarAngle() {
        activeZone = "FAR";
        setShooterAngle(ANGLE_FAR);
    }

    public void runDistanceShot(double distanceToGoal) {
        activeDistanceToGoal = distanceToGoal;

        // Distance влияет только на угол
        if (distanceToGoal <= CLOSE_DISTANCE_MAX) {
            setCloseAngle();
        } else {
            setFarAngle();
        }

        // Velocity всегда одна
        runClosedLoop_N(OPTIMAL_TARGET_VELOCITY);
    }

    public void setStopperInitial() {
        stopperServo.setPosition(STOPPER_INITIAL_POS);
    }

    public void setStopperShooting() {
        stopperServo.setPosition(STOPPER_SHOOT_POS);
    }

    public double getAngleSSStopperPosition() {
        return stopperServo.getPosition();
    }

    public double getShooterAnglePosition() {
        return activeAnglePosition;
    }

    public double getLeftAnglePosition() {
        return activeAnglePosition;
    }

    public double getRightAnglePosition() {
        if (RIGHT_SERVO_REVERSED) {
            return 1.0 - activeAnglePosition;
        }

        return activeAnglePosition;
    }

    public double getActiveDistanceToGoal() {
        return activeDistanceToGoal;
    }

    public String getActiveZone() {
        return activeZone;
    }

    private double clampAngle(double value) {
        return Math.max(MIN_ANGLE_POS, Math.min(MAX_ANGLE_POS, value));
    }
}