package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ShootingLogicHigh {
    private Servo gateServo;
    private DcMotorEx flywheelMotorLeft, flywheelMotorRight, intake_motor;
    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState { IDLE, SPIN_UP, WAIT_FOR_GATE, SHOOT_ALL, RESET }
    private FlywheelState flywheelState = FlywheelState.IDLE;

    static final double kS = 0.034;
    static final double kV = 1.0 / 2400.0;
    static final double kP = 0.003524;
    static final double VELOCITY_HIGH = 3000;
    static final double VELO_TOLERANCE = 20;

    private final double GATE_CLOSE_ANGLE = 0.75;
    private final double GATE_OPEN_ANGLE = 0.1;

    private final double GATE_WAIT_TIME =0.8;
    private int shotsRemaining = 0;
    private double targetVelocity = 0;

    public void init(HardwareMap hwMap) {
        gateServo = hwMap.get(Servo.class, "servoShooter");
        flywheelMotorLeft = hwMap.get(DcMotorEx.class, "flywheelMotorLeft");
        flywheelMotorRight = hwMap.get(DcMotorEx.class, "flywheelMotorRight");
        intake_motor = hwMap.get(DcMotorEx.class, "intake");

        flywheelMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelMotorRight.setDirection(DcMotorEx.Direction.FORWARD);

        flywheelMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        gateServo.setPosition(GATE_CLOSE_ANGLE);
    }

    public void update() {
        double currentVelocity = flywheelMotorLeft.getVelocity();
        double error = targetVelocity - currentVelocity;
        double power = 0.0;

        if (targetVelocity > 0) {
            power = (kV * targetVelocity) + (kS * Math.signum(targetVelocity)) + (kP * error);
        }
        power = Range.clip(power, 0.0, 1.0);

        flywheelMotorLeft.setPower(power);
        flywheelMotorRight.setPower(power);

        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0 || targetVelocity > 0) {
                    targetVelocity = VELOCITY_HIGH;
                    flywheelState = FlywheelState.SPIN_UP;
                } else {
                    targetVelocity = 0;
                    gateServo.setPosition(GATE_CLOSE_ANGLE);
                }
                break;

            case SPIN_UP:
                boolean speedReady = Math.abs(targetVelocity - currentVelocity) < VELO_TOLERANCE;
                if (shotsRemaining > 0 && speedReady) {
                    gateServo.setPosition(GATE_OPEN_ANGLE);
                    stateTimer.reset();
                    flywheelState = FlywheelState.WAIT_FOR_GATE;
                }
                break;

            case WAIT_FOR_GATE:
                if (stateTimer.seconds() > GATE_WAIT_TIME) {
                    intake_motor.setPower(-1);
                    stateTimer.reset();
                    flywheelState = FlywheelState.SHOOT_ALL;
                }
                break;

            case SHOOT_ALL:
                if (stateTimer.seconds() > 1) {
                    intake_motor.setPower(0);
                    gateServo.setPosition(GATE_CLOSE_ANGLE);
                    targetVelocity = 0;
                    shotsRemaining = 0;
                    stateTimer.reset();
                    flywheelState = FlywheelState.RESET;
                }
                break;

            case RESET:
                if (stateTimer.seconds() > 0.1) {
                    flywheelState = FlywheelState.IDLE;
                }
                break;
        }
    }

    public void fireShots(int count) {
        if (flywheelState == FlywheelState.IDLE || flywheelState == FlywheelState.SPIN_UP) {
            this.shotsRemaining = count;
        }
    }
    public void prepareShooter() {
        if (flywheelState == FlywheelState.IDLE) {
            targetVelocity = VELOCITY_HIGH;
            flywheelState = FlywheelState.SPIN_UP;
        }
    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }
}