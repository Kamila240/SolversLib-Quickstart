package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleops.cs;
import org.firstinspires.ftc.teamcode.teleops.cs;

import com.pedropathing.geometry.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "RamazanAuto", group = "Autonomous")
@Configurable
public class RamazanAuto extends OpMode {

    private Follower follower;
    private DcMotor intake1, intake2;
    private DcMotorEx flywheelL, flywheelR;
    private Servo stopper, turret1, turret2;
    private VoltageSensor voltageSensor;

    private TelemetryManager panelsTelemetry;
    private ElapsedTime pathTimer = new ElapsedTime();
    private int pathState = 0;
    private Paths paths;

    public static double flywheel_kV = 0.00039;
    public static double flywheel_kS = 0.063;
    public static double flywheel_kP = 0.0375;

    private final double SHOOTING_VELOCITY = 2500;
    private final double IDLE_VELOCITY = 1000;
    private double TARGET_VELOCITY = IDLE_VELOCITY;

    private org.firstinspires.ftc.teamcode.teleops.cs cs;
    private final double goalX = cs.BLUE_GOAL_X;
    private final double goalY = cs.BLUE_GOAL_Y;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(45.895, 6.105, Math.toRadians(180)));

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        flywheelL = hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheelR = hardwareMap.get(DcMotorEx.class, "shooter1");

        stopper = hardwareMap.get(Servo.class, "stopper");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelR.setDirection(DcMotorSimple.Direction.FORWARD);

        paths = new Paths(follower);

        updateTurret(follower.getPose());

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        updateTurret(follower.getPose());

        double currentV = flywheelL.getVelocity();
        double power = calculateFlywheelPower(currentV);

        flywheelL.setPower(power);
        flywheelR.setPower(power);

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("TargetV", TARGET_VELOCITY);
        panelsTelemetry.debug("CurrentV", currentV);
        panelsTelemetry.update(telemetry);
    }

    private boolean waitTimePassed(double ms) {
        return pathTimer.milliseconds() > ms;
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.reset();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                prepareShooting();
                if (waitTimePassed(350)) {
                    follower.followPath(paths.Intake1);
                    prepareIntake();
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    prepareShooting();
                    follower.followPath(paths.Shooting1);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && waitTimePassed(350)) {
                    follower.followPath(paths.Intake2);
                    prepareIntake();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    prepareShooting();
                    follower.followPath(paths.Shooting2);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && waitTimePassed(350)) {
                    follower.followPath(paths.Intake3);
                    prepareIntake();
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    prepareShooting();
                    follower.followPath(paths.Shooting3);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && waitTimePassed(350)) {
                    follower.followPath(paths.Intake4);
                    prepareIntake();
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    prepareShooting();
                    follower.followPath(paths.Shooting5);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() && waitTimePassed(350)) {
                    follower.followPath(paths.Intake6);
                    prepareIntake();
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    prepareShooting();
                    follower.followPath(paths.Shooting6);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() && waitTimePassed(350)) {
                    follower.followPath(paths.Leave);
                    setPathState(-1);
                }
                break;
        }
    }

    private void prepareShooting() {
        TARGET_VELOCITY = SHOOTING_VELOCITY;
        intake1.setPower(-1);
        intake2.setPower(-1);
        stopper.setPosition(cs.stopper_open);
    }

    private void prepareIntake() {
        TARGET_VELOCITY = IDLE_VELOCITY;
        intake1.setPower(-0.9);
        intake2.setPower(-0.9);
        stopper.setPosition(cs.stopper_closed);
    }

    double calculateFlywheelPower(double currentV) {
        double voltage = voltageSensor.getVoltage();
        double error = TARGET_VELOCITY - currentV;

        double power = (flywheel_kV * TARGET_VELOCITY)
                + (flywheel_kS * Math.signum(TARGET_VELOCITY))
                + (flywheel_kP * error) * (13.5 / voltage);

        return Range.clip(power, -1, 1);
    }

    void updateTurret(Pose pose) {
        double angleToGoal = Math.atan2(goalY - pose.getY(), goalX - pose.getX());
        double relativeAngle = Math.toDegrees(angleToGoal - pose.getHeading());

        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        double servoPos = Range.clip(0.5 + (Range.clip(relativeAngle, -120, 120) / 240.0), 0.0, 1.0);

        turret1.setPosition(servoPos);
        turret2.setPosition(servoPos);
    }

    public static class Paths {

        public PathChain Intake1, Shooting1, Intake2, Shooting2,
                Intake3, Shooting3, Intake4, Shooting5,
                Intake6, Shooting6, Leave;

        public Paths(Follower follower) {

            Intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(45.895, 6.105), new Pose(10.737, 6.105)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Shooting1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(10.737, 6.105), new Pose(45.895, 6.105)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Intake2 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(45.895, 6.105), new Pose(60.947, 39.184), new Pose(10.211, 35.842)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Shooting2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(10.211, 35.842), new Pose(45.895, 6.105)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Intake3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(45.895, 6.105), new Pose(11.158, 14.684)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Shooting3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(11.158, 14.684), new Pose(45.895, 6.105)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Intake4 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(45.895, 6.105), new Pose(10.553, 15.237), new Pose(10.158, 30.789)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120)).build();

            Shooting5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(10.158, 30.789), new Pose(50.368, 9.947)))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180)).build();

            Intake6 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(50.368, 9.947), new Pose(31.342, 9.658), new Pose(11.368, 7.368)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Shooting6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(11.368, 7.368), new Pose(45.895, 6.105)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Leave = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(45.895, 6.105), new Pose(37.053, 6.105)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();
        }
    }
}