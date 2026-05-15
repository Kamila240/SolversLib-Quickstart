package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    public static final double VELOCITY_IDLE = 0.0;
    public static final double VELOCITY_HIGH = 2000.0;
    public static final double VELOCITY_LOW  = 2500.0;

    public static final double GATE_OPEN_POS = 0.25;
    public static final double GATE_CLOSED_POS = 1.0;

    public static final double VELOCITY_TOLERANCE = 40.0;

    private final DcMotor intake1, intake2;
    private final DcMotorEx flywheelLeft;
    private final DcMotorEx flywheelRight;
    private final Servo servoShooter;

    private final double kS = 0.034;
    private final double kV = 1.0 / 2400.0;
    private final double kP = 0.004;

    private double targetVelocity = 0.0;
    private double appliedPower = 0.0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2= hardwareMap.get(DcMotor.class, "intake2");
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelMotorLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelMotorRight");
        servoShooter = hardwareMap.get(Servo.class, "servoShooter");

        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheelRight.setDirection(DcMotor.Direction.FORWARD);

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoShooter.setPosition(GATE_CLOSED_POS);
    }

    public void setIntake(double power) {
        intake1.setPower(power);
        intake2.setPower(power);

    }

    public void stopIntake() {
        intake1.setPower(0.0);
        intake2.setPower(0.0);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = Math.max(0.0, velocity);
    }

    public void setGateOpen(boolean open) {
        servoShooter.setPosition(open ? GATE_OPEN_POS : GATE_CLOSED_POS);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getLeftVelocity() {
        return Math.abs(flywheelLeft.getVelocity());
    }

    public double getRightVelocity() {
        return Math.abs(flywheelRight.getVelocity());
    }

    public double getAverageVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    public double getAppliedPower() {
        return appliedPower;
    }

    public boolean isAtSpeed() {
        return Math.abs(targetVelocity - getAverageVelocity()) <= VELOCITY_TOLERANCE;
    }

    @Override
    public void periodic() {
        double currentVelocity = getAverageVelocity();
        double error = targetVelocity - currentVelocity;

        double power =
                (kV * targetVelocity) +
                        (kS * Math.signum(targetVelocity)) +
                        (kP * error);

        appliedPower = Range.clip(power, 0.0, 1.0);

        if (targetVelocity <= 1.0) {
            appliedPower = 0.0;
            flywheelLeft.setPower(0.0);
            flywheelRight.setPower(0.0);
        } else {
            flywheelLeft.setPower(appliedPower);
            flywheelRight.setPower(appliedPower);
        }
    }
}