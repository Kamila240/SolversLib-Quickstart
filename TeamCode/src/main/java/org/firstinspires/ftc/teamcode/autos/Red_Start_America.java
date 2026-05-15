package org.firstinspires.ftc.teamcode.autos;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Configs.AutoAimConfig_Red;
import org.firstinspires.ftc.teamcode.Configs.TurretConfig_goal;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem_goal;

@Config
@Autonomous(name = "Red_Start_America_FIXED", group = "Autonomous")
public class Red_Start_America extends OpMode {

    private PedroDriveSubsystem drive;
    private ShooterSubsystem shooter;
    private TurretSubsystem_goal turret;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime burstTimer = new ElapsedTime();

    public static double AUTO_SHOT_VELOCITY = 2100.0;

    public static long INTAKE_TIME_LONG_MS = 1400;
    public static long INTAKE_TIME_SHORT_MS = 900;

    public static double STOPPER_TO_INTAKE_DELAY_MS = 100.0;
    public static double FEED_TIME_MS = 450.0;
    public static double RECOVER_TIME_MS = 90.0;
    public static double SPINUP_TIMEOUT_MS = 1100.0;

    private boolean autoAimEnabled = true;
    private boolean arrivalLatched = false;
    private boolean burstTimedOut = false;

    private enum BurstState {
        IDLE,
        SPINUP,
        OPEN_DELAY,
        FEED,
        RECOVER,
        FINISHED
    }

    private BurstState burstState = BurstState.IDLE;

    private boolean burstStarted = false;
    private int burstCount = 0;
    private int burstShotsDone = 0;
    private double burstTargetVelocity = AUTO_SHOT_VELOCITY;

    private enum AutoState {
        SHOOT,

        CYCLE1_GRAB,
        CYCLE1_RETURN,
        CYCLE1_SHOOT,

        CYCLE2_GRAB,
        CYCLE2_RETURN,
        CYCLE2_SHOOT,

        CYCLE3_GRAB,
        CYCLE3_RETURN,
        CYCLE3_SHOOT,

        LEAVE,
        DONE
    }

    private AutoState autoState = AutoState.SHOOT;

    private final Pose startPose = new Pose(85.31386292834891, 7.338785046728957, Math.toRadians(0));

    private final Pose pickup1Pose = new Pose(132.0622453431793, 8.182122676186278, Math.toRadians(0));

    private final Pose score1Pose = new Pose(84.06219713860378, 16.980944537782204, Math.toRadians(0));
    private final Pose score1controlPose = new Pose(114.13971345272954, 20.720162890473347);

    private final Pose pickup2Pose = new Pose(118.5654192169931, 34.948465720777996, Math.toRadians(0));
    private final Pose pickup2controlPose = new Pose(90.71801378527506, 39.48690139096235);
    private final Pose score2Pose = new Pose(84.05418428232898, 16.999985420937517, Math.toRadians(60));

    private final Pose pickup3Pose = new Pose(132.68909732250407, 47.98405354564465, Math.toRadians(85));
    private final Pose pickup3controlPose = new Pose(123.54687444727632, 24.202299857122846);
    private final Pose score3Pose = new Pose(84.14580755564167, 16.983484463528626, Math.toRadians(60));
    private final Pose score3controlPose = new Pose(123.95600384094203, 20.896152182156744);

    private final Pose leavePose = new Pose(84.06687138626634, 28.499874971412602, Math.toRadians(90));

    private PathChain grabPickup1;
    private PathChain scorePickup1;

    private PathChain grabPickup2;
    private PathChain scorePickup2;

    private PathChain grabPickup3;
    private PathChain scorePickup3;

    private PathChain leave;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        drive = new PedroDriveSubsystem(hardwareMap, telemetry, startPose, false);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem_goal(hardwareMap, telemetry);

        turret.setControlEnabled(true);
        turret.center();

        shooter.stopAll();
        shooter.closeStopper();

        buildPaths();

        autoState = AutoState.SHOOT;
        autoAimEnabled = true;
        arrivalLatched = false;
        burstStarted = false;
        burstState = BurstState.IDLE;
    }

    @Override
    public void start() {
        setState(AutoState.SHOOT);
    }

    @Override
    public void loop() {
        drive.update();

        updateShooterSpin();

        if (autoAimEnabled) {
            turret.aimAtFieldPoint(
                    drive.getPose(),
                    AutoAimConfig_Red.GOAL_X,
                    AutoAimConfig_Red.GOAL_Y
            );
        }

        turret.periodic();

        updateAuto();

        telemetry.addData("Auto State", autoState);
        telemetry.addData("Drive Busy", drive.isBusy());

        telemetry.addData("Burst State", burstState);
        telemetry.addData("Burst Done", burstShotsDone);
        telemetry.addData("Burst Timed Out", burstTimedOut);

        telemetry.addData("Shooter Target", shooter.getActiveTargetVelocity());
        telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
        //telemetry.addData("Shooter Filtered", shooter.getFilteredVelocity());
        telemetry.addData("Shooter Power", shooter.getShooterPower());
        telemetry.addData("Shooter Ready", shooter.isShooterReady());
        //telemetry.addData("Shooter Error", shooter.getError());
        telemetry.addData("Battery Voltage", shooter.getBatteryVoltage());

        telemetry.addData("Pose X", drive.getPose().getX());
        telemetry.addData("Pose Y", drive.getPose().getY());
        telemetry.addData("Pose Heading Deg", Math.toDegrees(drive.getPose().getHeading()));

        telemetry.update();
    }

    private void buildPaths() {
        grabPickup1 = drive.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, score1controlPose, score1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading())
                .build();

        grabPickup2 = drive.pathBuilder()
                .addPath(new BezierCurve(score1Pose, pickup2controlPose, pickup2Pose))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = drive.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, score2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading())
                .build();

        grabPickup3 = drive.pathBuilder()
                .addPath(new BezierCurve(score2Pose, pickup3controlPose, pickup3Pose))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, score3controlPose, score3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading())
                .build();

        leave = drive.pathBuilder()
                .addPath(new BezierLine(score3Pose, leavePose))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), leavePose.getHeading())
                .build();
    }

    private void updateShooterSpin() {
        if (autoState == AutoState.DONE) {
            return;
        }

        shooter.runClosedLoop_F(AUTO_SHOT_VELOCITY);
    }

    private void updateAuto() {
        switch (autoState) {

            case SHOOT:
                autoAimEnabled = true;
                TurretConfig_goal.OFFSET = 5.0;

                startBurst(1, AUTO_SHOT_VELOCITY);
                setState(AutoState.CYCLE1_GRAB);
                break;

            case CYCLE1_GRAB:
                if (updateBurst()) {
                    finishBurstKeepFlywheel();
                    startGrabPath(grabPickup1, 1.0);
                    setState(AutoState.CYCLE1_RETURN);
                }
                break;

            case CYCLE1_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup1, 0.0, 1.0);
                    setState(AutoState.CYCLE1_SHOOT);
                }
                break;

            case CYCLE1_SHOOT:
                autoAimEnabled = true;

                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE2_GRAB);
                }
                break;

            case CYCLE2_GRAB:
                if (updateBurst()) {
                    finishBurstKeepFlywheel();
                    startGrabPath(grabPickup2, 0.85);
                    setState(AutoState.CYCLE2_RETURN);
                }
                break;

            case CYCLE2_RETURN:
                if (handleCollectTotal(INTAKE_TIME_SHORT_MS)) {
                    startReturnPath(scorePickup2, 0.0, 1.0);
                    setState(AutoState.CYCLE2_SHOOT);
                }
                break;

            case CYCLE2_SHOOT:
                autoAimEnabled = true;

                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE3_GRAB);
                }
                break;

            case CYCLE3_GRAB:
                if (updateBurst()) {
                    finishBurstKeepFlywheel();
                    startGrabPath(grabPickup3, 1.0);
                    setState(AutoState.CYCLE3_RETURN);
                }
                break;

            case CYCLE3_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup3, 0.0, 1.0);
                    setState(AutoState.CYCLE3_SHOOT);
                }
                break;

            case CYCLE3_SHOOT:
                autoAimEnabled = true;

                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.LEAVE);
                }
                break;

            case LEAVE:
                if (updateBurst()) {
                    finishBurstAndStopShooter();
                    startLeavePath(leave, 1.0);
                    setState(AutoState.DONE);
                }
                break;

            case DONE:
                shooter.stopAll();
                autoAimEnabled = false;
                break;
        }
    }

    private void setState(AutoState newState) {
        autoState = newState;
        stateTimer.reset();
        arrivalLatched = false;
    }

    private void startGrabPath(PathChain path, double maxPower) {
        autoAimEnabled = false;

        shooter.closeStopper();
        shooter.feedIntake();

        drive.setMaxPower(maxPower);
        drive.followPath(path, true);

        stateTimer.reset();
        arrivalLatched = false;
    }

    private void startReturnPath(PathChain path, double turretOffsetDeg, double maxPower) {
        shooter.stopIntake();
        shooter.closeStopper();

        autoAimEnabled = true;
        TurretConfig_goal.OFFSET = turretOffsetDeg;

        drive.setMaxPower(maxPower);
        drive.followPath(path, true);

        stateTimer.reset();
        arrivalLatched = false;
    }

    private void startLeavePath(PathChain path, double maxPower) {
        autoAimEnabled = false;

        shooter.stopAll();

        drive.setMaxPower(maxPower);
        drive.followPath(path, true);

        stateTimer.reset();
        arrivalLatched = false;
    }

    private boolean handleCollectTotal(long totalMs) {
        shooter.feedIntake();

        return !drive.isBusy() && stateTimer.milliseconds() >= totalMs;
    }

    private boolean handleCollectExtraAfterDriveDone(long extraMs) {
        shooter.feedIntake();

        if (drive.isBusy()) {
            return false;
        }

        if (!arrivalLatched) {
            arrivalLatched = true;
            stateTimer.reset();
        }

        return stateTimer.milliseconds() >= extraMs;
    }

    private void startBurst(int count, double targetVelocity) {
        burstStarted = true;
        burstCount = count;
        burstShotsDone = 0;
        burstTargetVelocity = targetVelocity;
        burstState = BurstState.SPINUP;
        burstTimedOut = false;

        shooter.closeStopper();
        shooter.stopIntake();

        burstTimer.reset();
    }

    private boolean updateBurst() {
        if (!burstStarted) {
            return false;
        }

        shooter.runClosedLoop_F(burstTargetVelocity);

        switch (burstState) {

            case SPINUP:
                boolean ready = shooter.isReadyFor(burstTargetVelocity);
                boolean timeout = burstTimer.milliseconds() >= SPINUP_TIMEOUT_MS;

                if (ready || timeout) {
                    burstTimedOut = timeout && !ready;

                    shooter.openStopper();
                    shooter.stopIntake();

                    burstTimer.reset();
                    burstState = BurstState.OPEN_DELAY;
                }
                break;

            case OPEN_DELAY:
                shooter.openStopper();
                shooter.stopIntake();

                if (burstTimer.milliseconds() >= STOPPER_TO_INTAKE_DELAY_MS) {
                    shooter.feedIntake();

                    burstTimer.reset();
                    burstState = BurstState.FEED;
                }
                break;

            case FEED:
                shooter.openStopper();
                shooter.feedIntake();

                if (burstTimer.milliseconds() >= FEED_TIME_MS) {
                    shooter.stopIntake();
                    shooter.closeStopper();

                    burstTimer.reset();
                    burstState = BurstState.RECOVER;
                }
                break;

            case RECOVER:
                shooter.closeStopper();
                shooter.stopIntake();

                if (burstTimer.milliseconds() >= RECOVER_TIME_MS) {
                    burstShotsDone++;

                    if (burstShotsDone >= burstCount) {
                        burstState = BurstState.FINISHED;
                    } else {
                        burstState = BurstState.SPINUP;
                        burstTimer.reset();
                    }
                }
                break;

            case FINISHED:
                return true;

            case IDLE:
            default:
                break;
        }

        return burstState == BurstState.FINISHED;
    }

    private void finishBurstKeepFlywheel() {
        burstStarted = false;
        burstState = BurstState.IDLE;

        shooter.stopIntake();
        shooter.closeStopper();
    }

    private void finishBurstAndStopShooter() {
        burstStarted = false;
        burstState = BurstState.IDLE;

        shooter.stopIntake();
        shooter.closeStopper();
        shooter.stopFlywheels();
    }
}