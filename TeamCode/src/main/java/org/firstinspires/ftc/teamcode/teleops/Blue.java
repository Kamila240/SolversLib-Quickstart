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
@TeleOp(name = "Blue")
public class Blue extends LinearOpMode {

    public static double VELOCITY_THRESHOLD = 10.0;
    public static double flywheel_kV = 0.00039;
    public static double flywheel_kS = 0.063;
    public static double flywheel_kP = 0.039;
    public static double TARGET_VELOCITY = 0;
    public static double TURRET_MAX_STEP = 0.02;

    private Follower follower;
    private DcMotor intake1, intake2;
    private DcMotorEx flywheelL, flywheelR;
    private Servo stopper, turret1, turret2, hood;
    private DcMotor fl, fr, rl, rr;
    private VoltageSensor voltageSensor;

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

        intake1 = hardwareMap.dcMotor.get(cs.INTAKE_1);
        intake2 = hardwareMap.dcMotor.get(cs.INTAKE_2);
        flywheelL = hardwareMap.get(DcMotorEx.class, cs.FLYWHEEL_LEFT);
        flywheelR = hardwareMap.get(DcMotorEx.class, cs.FLYWHEEL_RIGHT);
        stopper = hardwareMap.servo.get(cs.STOPPER);
        turret1 = hardwareMap.servo.get(cs.TURRET_1);
        turret2 = hardwareMap.servo.get(cs.TURRET_2);
        hood = hardwareMap.servo.get(cs.HOOD);

        fl = hardwareMap.dcMotor.get(cs.FRONT_LEFT);
        fr = hardwareMap.dcMotor.get(cs.FRONT_RIGHT);
        rl = hardwareMap.dcMotor.get(cs.REAR_LEFT);
        rr = hardwareMap.dcMotor.get(cs.REAR_RIGHT);

        flywheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);

        stopper.setPosition(cs.stopper_closed);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        turret1.setPosition(cs.turret_center);
        turret2.setPosition(cs.turret_center);
        hood.setPosition(0.0);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            driveDriverCentric();
            currentPose = follower.getPose();

            goalX = cs.BLUE_GOAL_X;
            goalY = cs.BLUE_GOAL_Y;

            double distanceToGoal = Math.hypot(goalX - currentPose.getX(), goalY - currentPose.getY());
            TARGET_VELOCITY = calculateTargetVelocityFromDistanceInches(distanceToGoal);

            double hoodPos = calculateHoodPosition(distanceToGoal);
            hood.setPosition(hoodPos);

            if ( gamepad1.right_trigger > 0.5  || gamepad2.right_trigger >0.5) {
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
                    updateTurret(currentPose);
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
            if (gamepad2.a) {
                hood.setPosition(0.0);
            }
            else if (gamepad2.b) {
                hood.setPosition(0.5);
            } else if (gamepad2.y) {
                hood.setPosition(0.75);
            }
            else if (gamepad2.x) {
                hood.setPosition(1.0);
            }
            telemetry.addData("Shooter State", currentShooterState.name());
            telemetry.addData("Target Velocity", TARGET_VELOCITY);
            telemetry.addData("Current Velocity", flywheelL.getVelocity());
            telemetry.addData("Distance", distanceToGoal);
            telemetry.addData("Hood Pos", hoodPos);
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    void driveDriverCentric() {
        double y = gamepad1.left_stick_x;
        double x = gamepad1.left_stick_y * 1.1;
        double rx = gamepad1.right_stick_x;

        double heading = follower.getPose().getHeading();
        heading += Math.toRadians(-90);

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
            power = ( (gamepad1.right_bumper || gamepad2.right_bumper) ? -0.8 : (( gamepad1.left_bumper || gamepad2.left_bumper) ? 0.8 : 0.0));
        }
        intake1.setPower(power);
        intake2.setPower(power);
    }

    double calculateFlywheelPower(double currentV) {
        double voltage = voltageSensor.getVoltage();
        double error = TARGET_VELOCITY - currentV;
        return Range.clip((flywheel_kV * TARGET_VELOCITY) + (flywheel_kS * Math.signum(TARGET_VELOCITY)) + (flywheel_kP * error) * (13.5 / voltage), -1, 1);
    }

    double calculateTargetVelocityFromDistanceInches(double x) {
        if (x > 135) return 2250;
        return 0.0000616617 * Math.pow(x, 3) + 0.00845565 * Math.pow(x, 2) + 0.569078 * x + 1550;
    }

    double calculateHoodPosition(double x) {
        if (x > 72.3) return 0.0;

        double pos = 0.000007057545 * Math.pow(x, 4)
                - 0.002095294 * Math.pow(x, 3)
                + 0.223944 * Math.pow(x, 2)
                - 10.19258 * x
                + 168.11239;
        return Range.clip(pos, 0.0, 1.0);
    }

    void updateTurret(Pose pose) {
        double vx = follower.getVelocity().getXComponent();
        double vy = follower.getVelocity().getYComponent();

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();
        double distance = Math.hypot(dx, dy);

        double projectileSpeed = 100.0;
        double tof = distance / projectileSpeed;

        double virtualGoalX = goalX - vx * tof;
        double virtualGoalY = goalY - vy * tof;

        double angleToGoal = Math.atan2(
                virtualGoalY - pose.getY(),
                virtualGoalX - pose.getX()
        );

        double relativeAngle = Math.toDegrees(angleToGoal - pose.getHeading());

        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        double targetServoPos = Range.clip(
                cs.turret_center +
                        (Range.clip(relativeAngle, -135, 135) / cs.turret_range),
                0.0, 1.0
        );

        double currentTurretPos = turret1.getPosition();
        double newPos;

        if (Math.abs(targetServoPos - currentTurretPos) > TURRET_MAX_STEP) {
            if (targetServoPos > currentTurretPos) {
                newPos = currentTurretPos + TURRET_MAX_STEP;
            } else {
                newPos = currentTurretPos - TURRET_MAX_STEP;
            }
        } else {
            newPos = targetServoPos;
        }

        turret1.setPosition(newPos);
        turret2.setPosition(newPos);
    }
}