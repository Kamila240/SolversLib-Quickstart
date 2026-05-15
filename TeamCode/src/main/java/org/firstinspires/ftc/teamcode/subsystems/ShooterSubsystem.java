package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx flywheelL;
    private final DcMotorEx flywheelR;
    private final DcMotor intake1;
    private final DcMotor intake2;
    private final Servo stopper;
    private final VoltageSensor batterySensor;

    public static double NORMAL_TARGET_VELOCITY = 1700.0;
    public static double FAR_TARGET_VELOCITY = 2250.0;
    public static double READY_TOLERANCE = 130.0;

    public static double SHOOTER_kV_N = 0.0008555;
    public static double SHOOTER_kS_N=  0.205;
    public static double SHOOTER_kP_N = 0.09; //0.00996

    public static double SHOOTER_kV_F = 0.0068055;
    public static double SHOOTER_kS_F=  0.805;
    public static double SHOOTER_kP_F = 0.09; //0.00996

    public static double STOPPER_CLOSED = 0.5;
    public static double STOPPER_OPEN = 0.7;

    public static double INTAKE_FEED_POWER = 1.0;
    public static double INTAKE_MANUAL_POWER = 1.0;

    public static boolean USE_VOLTAGE_COMP = true;
    public static boolean REVERSE_LEFT_SHOOTER = true;
    public static boolean REVERSE_RIGHT_SHOOTER = false;
    public static boolean REVERSE_INTAKE1 = false;
    public static boolean REVERSE_INTAKE2 = false;

    private double initVoltage = 12.0;
    private double shooterVelocity = 0.0;
    private double shooterPower = 0.0;
    private double activeTargetVelocity = 0.0;
    private boolean shooterReady = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelL = hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheelR = hardwareMap.get(DcMotorEx.class, "shooter1");
        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        stopper = hardwareMap.servo.get("stopper");
        batterySensor = hardwareMap.voltageSensor.iterator().next();

        initVoltage = getBatteryVoltage();
        applyDirections();
        stopAll();
    }

    public void runClosedLoop_N(double targetVelocity) {
        applyDirections();

        shooterVelocity = Math.abs(flywheelL.getVelocity());
        shooterPower = calculateShooterPower_N(targetVelocity, shooterVelocity);
        shooterReady = shooterVelocity >= (targetVelocity - READY_TOLERANCE);
        activeTargetVelocity = targetVelocity;

        setFlywheelPower(shooterPower);
    }

    public void runClosedLoop_F(double targetVelocity) {
        applyDirections();

        shooterVelocity = Math.abs(flywheelL.getVelocity());
        shooterPower = calculateShooterPower_F(targetVelocity, shooterVelocity);
        shooterReady = shooterVelocity >= (targetVelocity - READY_TOLERANCE);
        activeTargetVelocity = targetVelocity;

        setFlywheelPower(shooterPower);
    }

    public void runClosedLoop(double targetVelocity, boolean farShot) {
        if (farShot) {
            runClosedLoop_F(targetVelocity);
        } else {
            runClosedLoop_N(targetVelocity);
        }
    }

    public void stopFlywheels() {
        setFlywheelPower(0.0);
        shooterPower = 0.0;
        activeTargetVelocity = 0.0;
        shooterReady = false;
    }

    public void openStopper() {
        stopper.setPosition(STOPPER_OPEN);
    }

    public void closeStopper() {
        stopper.setPosition(STOPPER_CLOSED);
    }

    public void feedIntake() {
        applyDirections();
        setIntakePower(-INTAKE_FEED_POWER);
    }

    public void reverseIntake() {
        applyDirections();
        setIntakePower(INTAKE_MANUAL_POWER);
    }

    public void stopIntake() {
        setIntakePower(0.0);
    }

    public void stopAll() {
        stopFlywheels();
        stopIntake();
        closeStopper();
    }

    public boolean isReadyFor(double targetVelocity) {
        return shooterVelocity >= (targetVelocity - READY_TOLERANCE);
    }

    public double getShooterVelocity() {
        return shooterVelocity;
    }

    public double getShooterPower() {
        return shooterPower;
    }

    public boolean isShooterReady() {
        return shooterReady;
    }

    public double getActiveTargetVelocity() {
        return activeTargetVelocity;
    }

    public double getBatteryVoltage() {
        double v = batterySensor.getVoltage();
        return v > 0.0 ? v : 12.0;
    }

    public double getStopperPosition() {
        return stopper.getPosition();
    }

    private double calculateShooterPower_N(double targetVelocity, double currentVelocity) {
        double rawPower =
                SHOOTER_kV_N * targetVelocity +
                        SHOOTER_kS_N * Math.signum(targetVelocity) +
                        SHOOTER_kP_N * (targetVelocity - currentVelocity);

        if (USE_VOLTAGE_COMP) {
            double currentVoltage = getBatteryVoltage();
            rawPower *= (initVoltage / currentVoltage);
        }

        return clamp(rawPower, 0.0, 1.0);
    }

    private double calculateShooterPower_F(double targetVelocity, double currentVelocity) {
        double rawPower =
                SHOOTER_kV_F * targetVelocity +
                        SHOOTER_kS_F * Math.signum(targetVelocity) +
                        SHOOTER_kP_F * (targetVelocity - currentVelocity);

        if (USE_VOLTAGE_COMP) {
            double currentVoltage = getBatteryVoltage();
            rawPower *= (initVoltage / currentVoltage);
        }

        return clamp(rawPower, 0.0, 1.0);
    }

    private void applyDirections() {
        flywheelL.setDirection(
                REVERSE_LEFT_SHOOTER ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD
        );
        flywheelR.setDirection(
                REVERSE_RIGHT_SHOOTER ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD
        );

        intake1.setDirection(
                REVERSE_INTAKE1 ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD
        );
        intake2.setDirection(
                REVERSE_INTAKE2 ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD
        );
    }

    private void setFlywheelPower(double power) {
        flywheelL.setPower(power);
        flywheelR.setPower(power);
    }

    private void setIntakePower(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}