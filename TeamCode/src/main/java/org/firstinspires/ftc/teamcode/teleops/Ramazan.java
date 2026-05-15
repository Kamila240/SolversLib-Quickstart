package org.firstinspires.ftc.teamcode.teleops;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "aicitizens I need this")
public class Ramazan extends LinearOpMode {

    // Tuning variables
    public static double VELOCITY_THRESHOLD = 10.0;
    public static double flywheel_kV = 0.00039;
    public static double flywheel_kS = 0.063;
    public static double flywheel_kP = 0.0345;


    public static double TARGET_VELOCITY = 0;

    // Turret tuning variables
    public static double TURRET_OFFSET_RIGHT = -10.0;
    public static double TURRET_OFFSET_LEFT = 10.0;

    private Follower follower;
    private DcMotor intake1, intake2;
    private DcMotorEx flywheelL, flywheelR;
    private Servo stopper, turret1, turret2;
    private DcMotor fl, fr, rl, rr;
    private VoltageSensor voltageSensor;

    private boolean isBlue = true;
    private Pose currentPose;
    private double goalX, goalY;

    private boolean isFeedingReady = false;

    private enum ShooterState { IDLE, FEEDING }
    private ShooterState currentShooterState = ShooterState.IDLE;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(cs.getStartPose());

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        flywheelL = hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheelR = hardwareMap.get(DcMotorEx.class, "shooter1");
        stopper = hardwareMap.servo.get("stopper");
        turret1 = hardwareMap.servo.get("turret1");
        turret2 = hardwareMap.servo.get("turret2");

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        rl = hardwareMap.dcMotor.get("rl");
        rr = hardwareMap.dcMotor.get("rr");

        flywheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);

        stopper.setPosition(cs.stopper_closed);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        turret1.setPosition(0.5);
        turret2.setPosition(0.5);
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left) isBlue = true;
            if (gamepad1.dpad_right) isBlue = false;
            telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            driveDriverCentric();
            currentPose = follower.getPose();

            goalX = isBlue ? cs.BLUE_GOAL_X : cs.RED_GOAL_X;
            goalY = isBlue ? cs.BLUE_GOAL_Y : cs.RED_GOAL_Y;

            double distanceToGoal = Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());
            TARGET_VELOCITY = calculateTargetVelocityFromDistanceInches(distanceToGoal);

            if (gamepad1.left_trigger > 0.5) {
                currentShooterState = ShooterState.IDLE;
            } else if (gamepad1.right_trigger > 0.5) {
                currentShooterState = ShooterState.FEEDING;
            } else {
                currentShooterState = ShooterState.IDLE;
            }

            handleIntake();

            switch (currentShooterState) {
                case IDLE:
                    isFeedingReady = false;
                    flywheelL.setPower(0);
                    flywheelR.setPower(0);
                    stopper.setPosition(cs.stopper_closed);
                    turret1.setPosition(0.5);
                    turret2.setPosition(0.5);
                    break;

                case FEEDING:
                    stopper.setPosition(cs.stopper_open);
                    updateTurret(currentPose);
                    double currentVelocity = flywheelL.getVelocity();
                    double power = calculateFlywheelPower(currentVelocity);
                    flywheelL.setPower(power);
                    flywheelR.setPower(power);

                    if (Math.abs(TARGET_VELOCITY - currentVelocity) < VELOCITY_THRESHOLD) {
                        isFeedingReady = true;
                    }

                    break;
            }

            telemetry.addData("Shooter State", currentShooterState.name());
            telemetry.addData("Target Velocity", TARGET_VELOCITY);
            telemetry.addData("Current Velocity", flywheelL.getVelocity());
            telemetry.addData("Feeding Ready", isFeedingReady);
            telemetry.update();
        }
    }

    void driveDriverCentric() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double allianceOffset = isBlue ? Math.toRadians(-90) : Math.toRadians(90);

        double heading = follower.getPose().getHeading() + allianceOffset;
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        fl.setPower((rotY + rotX + rx) / denom);
        rl.setPower((rotY - rotX + rx) / denom);
        fr.setPower((rotY - rotX - rx) / denom);
        rr.setPower((rotY + rotX - rx) / denom);
    }

    void handleIntake() {
        double power;
        if (currentShooterState == ShooterState.FEEDING && isFeedingReady) {
            power = -1.0;
        } else {
            power = (gamepad1.right_bumper ? -0.8 : (gamepad1.left_bumper ? 0.8 : 0.0));
        }
        intake1.setPower(power);
        intake2.setPower(power);
    }

    double calculateFlywheelPower(double currentV) {
        double voltage = voltageSensor.getVoltage();
        double error = TARGET_VELOCITY - currentV;
        return Range.clip((flywheel_kV * TARGET_VELOCITY) + (flywheel_kS * Math.signum(TARGET_VELOCITY)) + (flywheel_kP * error)*(13.5/voltage), -1, 1);
    }


    double calculateTargetVelocityFromDistanceInches(double x) {
        if (x > 135) {
            return 2100;
        }
        return 0.0000616617 * Math.pow(x, 3)
                + 0.00845565 * Math.pow(x, 2)
                + 0.569078 * x
                + 1550;
    }

    void updateTurret(Pose pose) {
        double angleToGoal = Math.atan2(goalY - pose.getY(), goalX - pose.getX());
        double relativeAngle = Math.toDegrees(angleToGoal - pose.getHeading());

        double headingDeg = Math.toDegrees(pose.getHeading());
        while (headingDeg > 180) headingDeg -= 360;
        while (headingDeg < -180) headingDeg += 360;

        double offset = 0;
        if (headingDeg >= 170 || headingDeg <= -90) {
            offset = TURRET_OFFSET_RIGHT;
        } else if (headingDeg >= 0 && headingDeg <= 120) {
            offset = TURRET_OFFSET_LEFT;
        }

        relativeAngle += offset;

        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        double servoPos = Range.clip(0.5 + (Range.clip(relativeAngle, -120, 120) / 240.0), 0.0, 1.0);
        turret1.setPosition(servoPos);
        turret2.setPosition(servoPos);
    }
}