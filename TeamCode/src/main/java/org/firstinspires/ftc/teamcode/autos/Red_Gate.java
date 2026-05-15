package org.firstinspires.ftc.teamcode.autos;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Configs.AutoAimConfig_Red;
import org.firstinspires.ftc.teamcode.Configs.TurretConfig_goal;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem_goal;

@Autonomous(name = "Red_Gate", group = "Autonomous")
public class Red_Gate extends OpMode {

    private PedroDriveSubsystem drive;
    private ShooterSubsystem shooter;
    private TurretSubsystem_goal turret;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime burstTimer = new ElapsedTime();

    private boolean autoAimEnabled = true;

    private boolean arrivalLatched = false;

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
    private double burstTargetVelocity = 0.0;

    private enum AutoState {

        SCOREPRELOAD,
        SHOOT,

        CYCLE1_GRAB,
        CYCLE1_RETURN,
        CYCLE1_SHOOT,

        CYCLE2_GRAB,
        CYCLE2_OPENGATE,
        CYCLE2_RETURN,
        CYCLE2_SHOOT,

        CYCLE3_GRAB,
        CYCLE3_RETURN,
        CYCLE3_SHOOT,

        CYCLE4_GRAB,
        CYCLE4_RETURN,
        CYCLE4_SHOOT,

        CYCLE5_GRAB,
        CYCLE5_RETURN,
        CYCLE5_SHOOT,

        CYCLE6_GRAB,
        CYCLE6_RETURN,
        CYCLE6_SHOOT,

        STOPALL,

        DONE
    }

    private AutoState autoState = AutoState.SCOREPRELOAD;

    private static final double AUTO_SHOT_VELOCITY = 1800.0;

    private static final long INTAKE_TIME_LONG_MS = 1500;
    private static final long INTAKE_TIME_SHORT_MS = 1250;

    private static final double STOPPER_TO_INTAKE_DELAY_MS = 80.0;
    private static final double FEED_TIME_MS = 800.0;
    private static final double RECOVER_TIME_MS = 150.0;

    private final Pose startPose = new Pose(126.08878504672899, 113.35358255451712, Math.toRadians(0));
    private final Pose scorePose = new Pose(98.59847155373674, 92.86889039572604, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(116.64034000026862, 81.81282000597623, Math.toRadians(0));
    private final Pose pickup1controlPose = new Pose(97.85981308411213, 78.68457943925235);
    private final Pose score1Pose = new Pose(93.02566639072563, 82.43821379068126, Math.toRadians(-50));
    private final Pose pickup2Pose = new Pose(117.7009168510594, 58.55865626733419, Math.toRadians(0));
    private final Pose pickup2controlPose = new Pose(111.39989598226322, 52.73130107262142);
    private final Pose openGatePose = new Pose(127.0058900833834, 68.72926460816416, Math.toRadians(0));
    //private final Pose openGatecontrolPose = new Pose(116.51267137999399, 67.26428754055293);
    private final Pose score2Pose = new Pose(92.87903685793204, 82.42586277449269, Math.toRadians(-45));
    private final Pose pickup3Pose = new Pose(130.7518067186545, 58.26606251633731, Math.toRadians(0));
    private final Pose eat1Pose = new Pose(133.168689368485, 52.59696671544898, Math.toRadians(35));
    private final Pose eat1controlPose = new Pose(122.84615147036102, 59.80690402399284);
    //private final Pose pickup3controlPose2 = new Pose(114.05218191290383, 62.26847043357698);
    private final Pose score3Pose = new Pose(93.04741002454598, 82.4130734067725, Math.toRadians(-45));
    private final Pose score3controlPose = new Pose(112.8712895719049, 58.04396087107959);
    private final Pose pickup4Pose = new Pose(130.70873316062767, 58.2779314450563, Math.toRadians(0));
    private final Pose eat2Pose = new Pose(133.19701905544156, 52.56864584723839, Math.toRadians(35));
    private final Pose eat2controlPose = new Pose(122.79789168435862, 59.90888054645886);
    //private final Pose pickup4controlPose2 = new Pose(114.07394386673013, 62.22322828261221);
    private final Pose score4Pose = new Pose(93.14714295648598, 82.34456274163232, Math.toRadians(-45));
    private final Pose score4controlPose = new Pose(112.93532088135319, 58.081214886335665);
    private final Pose pickup5Pose = new Pose(130.73281538933009, 39.975212210276005, Math.toRadians(10));
    private final Pose pickup5controlPose = new Pose(101.60547761527563, 17.180915513337364);
    private final Pose score5Pose = new Pose(93.0175765926528, 82.25315177314963, Math.toRadians(-45));
    private final Pose pickup6Pose = new Pose(133.66432317320107, 13.756825857747259, Math.toRadians(-90));
    private final Pose pickup6controlPose = new Pose(138.6629903813693, 62.01706046653878);
    private final Pose score6Pose = new Pose(78.78190564654645, 100.95858127175111, Math.toRadians(-60));



    /*private final Pose score1controlPose = new Pose(102.11682242990652, 69.36448598130843);
    private final Pose score2controlPose = new Pose(106.39719626168224, 62.065420560747654);

    private final Pose pickup1Pose = new Pose(120.44859813084112, 59.14018691588784, Math.toRadians(0));
    private final Pose pickup1controlPose = new Pose(92.41121495327103, 56.74766355140187);

    private final Pose pickup2Pose = new Pose(128.84018691588785, 63.72897196261683, Math.toRadians(28));
    private final Pose pickup2controlPose = new Pose(105.99065420560748, 62.54205607476634);

    private final Pose openGate2ControlPose = new Pose(124.0, 66.0, Math.toRadians(15));
    private final Pose openGate2Pose = new Pose(118.0, 72.0, Math.toRadians(0));*/

    private Path scorePreload;

    private PathChain grabPickup1;
    private PathChain scorePickup1;
    private PathChain eat1;
    private PathChain eat2;
    private PathChain grabPickup2;
    private PathChain openGate;
    private PathChain scorePickup2;
    private PathChain grabPickup3;
    private PathChain scorePickup3;
    private PathChain grabPickup4;
    private PathChain scorePickup4;
    private PathChain grabPickup5;
    private PathChain scorePickup5;
    private PathChain grabPickup6;
    private PathChain scorePickup6;

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

        autoState = AutoState.SCOREPRELOAD;
        autoAimEnabled = true;
        arrivalLatched = false;
    }

    @Override
    public void start() {
        setState(AutoState.SCOREPRELOAD);
    }

    @Override
    public void loop() {
        drive.update();

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
        telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
        telemetry.addData("Shooter Ready", shooter.isShooterReady());
        telemetry.addData("Pose X", drive.getPose().getX());
        telemetry.addData("Pose Y", drive.getPose().getY());
        telemetry.addData("Pose Heading Deg", Math.toDegrees(drive.getPose().getHeading()));
        telemetry.update();
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = drive.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup1controlPose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = drive.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, score1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = drive.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2controlPose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        openGate = drive.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, openGatePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), openGatePose.getHeading())
                .build();

        scorePickup2 = drive.pathBuilder()
                .addPath(new BezierLine(openGatePose, score2Pose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), score2Pose.getHeading())
                .build();

        grabPickup3 = drive.pathBuilder()
                .addPath(new BezierLine(score2Pose,pickup3Pose))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        eat1 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, eat1controlPose,eat1Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), eat1Pose.getHeading())
                .build();

        scorePickup3 = drive.pathBuilder()
                .addPath(new BezierCurve(eat1Pose,score3controlPose, score3Pose))
                .setLinearHeadingInterpolation(eat1Pose.getHeading(), score3Pose.getHeading())
                .build();

        grabPickup4 = drive.pathBuilder()
                .addPath(new BezierLine(score3Pose,pickup4Pose))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), pickup4Pose.getHeading())
                .build();

        eat1 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup4Pose, eat2controlPose,eat2Pose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), eat2Pose.getHeading())
                .build();

        scorePickup4 = drive.pathBuilder()
                .addPath(new BezierCurve(eat2Pose,score4controlPose, score4Pose))
                .setLinearHeadingInterpolation(eat2Pose.getHeading(), score4Pose.getHeading())
                .build();

        grabPickup5 = drive.pathBuilder()
                .addPath(new BezierCurve(score4Pose, pickup5controlPose, pickup5Pose))
                .setLinearHeadingInterpolation(score4Pose.getHeading(), pickup5Pose.getHeading())
                .build();

        scorePickup5 = drive.pathBuilder()
                .addPath(new BezierLine(pickup5Pose, score5Pose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), score5Pose.getHeading())
                .build();

        grabPickup6 = drive.pathBuilder()
                .addPath(new BezierCurve(score5Pose, pickup6controlPose, pickup6Pose))
                .setLinearHeadingInterpolation(score5Pose.getHeading(), pickup6Pose.getHeading())
                .build();

        scorePickup6 = drive.pathBuilder()
                .addPath(new BezierLine(pickup5Pose, score6Pose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), score6Pose.getHeading())
                .build();
    }

    private void updateAuto() {
        switch (autoState) {

            case SCOREPRELOAD:
                autoAimEnabled = true;
                //TurretConfig_goal.OFFSET = -2.0;
                drive.setMaxPower(1.0);
                shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                drive.followPath(scorePreload);
                setState(AutoState.SHOOT);
                break;

            case SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE1_GRAB);
                }
                break;

            case CYCLE1_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();

                    startGrabPath(grabPickup1, 1.0);
                    setState(AutoState.CYCLE1_RETURN);
                }
                break;

            case CYCLE1_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup1, 0.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE1_SHOOT);
                }
                break;

            case CYCLE1_SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE2_GRAB);
                }
                break;

            case CYCLE2_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    startGrabPath(grabPickup2, 0.75);
                    setState(AutoState.CYCLE2_OPENGATE);
                }
                break;

            case CYCLE2_OPENGATE:
                if (handleCollectExtraAfterDriveDone(INTAKE_TIME_SHORT_MS)) {
                    shooter.stopIntake();
                    autoAimEnabled = false;
                    drive.setMaxPower(0.75);
                    drive.followPath(openGate, true);
                    setState(AutoState.CYCLE2_RETURN);
                }
                break;

            case CYCLE2_RETURN:
                if (!drive.isBusy()) {
                    startReturnPath(scorePickup2, 0.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE2_SHOOT);
                }
                break;

            case CYCLE2_SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE3_GRAB);
                }
                break;

            /*case CYCLE2_SHOOT:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    setState(AutoState.DONE);
                }
                break;*/

            case CYCLE3_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();

                    startGrabPath(grabPickup3, 1.0);
                    setState(AutoState.CYCLE3_RETURN);
                }
                break;

            case CYCLE3_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup3, 0.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE3_SHOOT);
                }
                break;

            case CYCLE3_SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE4_GRAB);
                }
                break;
            case CYCLE4_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();

                    startGrabPath(grabPickup4, 1.0);
                    setState(AutoState.CYCLE4_RETURN);
                }
                break;

            case CYCLE4_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup4, 0.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE4_SHOOT);
                }
                break;

            case CYCLE4_SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE5_GRAB);
                }
                break;

            case CYCLE5_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();

                    startGrabPath(grabPickup5, 1.0);
                    setState(AutoState.CYCLE5_RETURN);
                }
                break;

            case CYCLE5_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup5, 0.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE5_SHOOT);
                }
                break;

            case CYCLE5_SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE6_GRAB);
                }
                break;

            case CYCLE6_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();

                    startGrabPath(grabPickup6, 1.0);
                    setState(AutoState.CYCLE6_RETURN);
                }
                break;

            case CYCLE6_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup6, 0.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE6_SHOOT);
                }
                break;

            case CYCLE6_SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.STOPALL);
                }
                break;

            case STOPALL:
                if (updateBurst()) {
                    stopBurstAndShooter();
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

        shooter.stopFlywheels();
        shooter.closeStopper();
        shooter.feedIntake();

        drive.setMaxPower(maxPower);
        drive.followPath(path, true);

        stateTimer.reset();
        arrivalLatched = false;
    }

    private void startReturnPath(PathChain path, double turretOffsetDeg, double maxPower) {
        shooter.stopIntake();

        autoAimEnabled = true;
        TurretConfig_goal.OFFSET = turretOffsetDeg;

        drive.setMaxPower(maxPower);
        drive.followPath(path, true);
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

    private void startBurst(int count, double targetVelocity) { //int count, double targetVelocity
        burstStarted = true;
        burstCount = count;
        burstShotsDone = 0;
        burstTargetVelocity = targetVelocity;
        burstState = BurstState.SPINUP;

        shooter.closeStopper();
        shooter.stopIntake();
        burstTimer.reset();
    }

    private boolean updateBurst() {
        if (!burstStarted) return false;

        shooter.runClosedLoop_N(burstTargetVelocity);

        switch (burstState) {
            case SPINUP:
                if (shooter.isReadyFor(burstTargetVelocity)) {
                    shooter.openStopper();
                    burstTimer.reset();
                    burstState = BurstState.OPEN_DELAY;
                }
                break;

            case OPEN_DELAY:
                if (burstTimer.milliseconds() >= STOPPER_TO_INTAKE_DELAY_MS) {
                    shooter.feedIntake();
                    burstTimer.reset();
                    burstState = BurstState.FEED;
                }
                break;

            case FEED:
                if (burstTimer.milliseconds() >= FEED_TIME_MS) {
                    shooter.stopIntake();
                    shooter.closeStopper();
                    burstTimer.reset();
                    burstState = BurstState.RECOVER;
                }
                break;

            case RECOVER:
                if (burstTimer.milliseconds() >= RECOVER_TIME_MS) {
                    burstShotsDone++;

                    if (burstShotsDone >= burstCount) {
                        burstState = BurstState.FINISHED;
                    } else {
                        burstState = BurstState.SPINUP;
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

    private void stopBurstAndShooter() {
        burstStarted = false;
        burstState = BurstState.IDLE;

        shooter.stopIntake();
        shooter.closeStopper();
        shooter.stopFlywheels();
    }
}