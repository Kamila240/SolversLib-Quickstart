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
import org.firstinspires.ftc.teamcode.OpModePoseStore;
import org.firstinspires.ftc.teamcode.Configs.AutoAimConfig_Red;
import org.firstinspires.ftc.teamcode.Configs.TurretConfig_goal;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem_goal;

@Autonomous(name = "Red_Start", group = "Autonomous")
public class Red_Start extends OpMode {

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
    private double burstTargetVelocity = 2100.0;

    private enum AutoState {

        SCOREPRELOAD,
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

        CYCLE4_GRAB,
        CYCLE4_RETURN,
        CYCLE4_SHOOT,

        CYCLE5_GRAB,
        CYCLE5_RETURN,
        CYCLE5_SHOOT,

        CYCLE6_GRAB,
        CYCLE6_RETURN,
        CYCLE6_SHOOT,

        LEAVE,

        STOPALL,

        DONE
    }

    private AutoState autoState = AutoState.SHOOT;

    private static final double AUTO_SHOT_VELOCITY = 2100.0;

    private static final long INTAKE_TIME_LONG_MS = 1200;
    private static final long INTAKE_TIME_SHORT_MS = 950;

    private static final double STOPPER_TO_INTAKE_DELAY_MS = 120.0;
    private static final double FEED_TIME_MS = 800.0;
    private static final double RECOVER_TIME_MS = 150.0;

    private final Pose startPose = new Pose(85.31386292834891, 7.338785046728957, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(132.0622453431793, 8.182122676186278, Math.toRadians(0));



    private final Pose score1Pose = new Pose(84.06219713860378, 16.980944537782204, Math.toRadians(0));
    private final Pose score1controlPose = new Pose(114.13971345272954, 20.720162890473347);


    private final Pose pickup2Pose = new Pose(118.5654192169931, 34.948465720777996, Math.toRadians(0));
    private final Pose pickup2controlPose = new Pose(90.71801378527506, 39.48690139096235);
    private final Pose score2Pose = new Pose(84.05418428232898, 16.999985420937517, Math.toRadians(60));


    private final Pose pickup3Pose = new Pose(132.23193221346978, 40.490284075239664, Math.toRadians(85));
    private final Pose pickup3controlPose = new Pose(123.54687444727632, 24.202299857122846);
    private final Pose score3Pose = new Pose(84.14580755564167, 16.983484463528626, Math.toRadians(60));
    private final Pose score3controlPose = new Pose(123.95600384094203, 20.896152182156744);


    private final Pose pickup4Pose = new Pose(133.44047816435787, 41.47466731275019, Math.toRadians(0));
    private final Pose pickup4controlPose = new Pose(91.89945127121474, 48.049169346083325);
    private final Pose score4Pose = new Pose(84.04934851317876, 17.108108407556596, Math.toRadians(90));
    private final Pose score4controlPose = new Pose(108.0062061736593, 31.40159035236524);


    private final Pose pickup5Pose = new Pose(132.2266355140187, 40.33411214953273, Math.toRadians(85));
    private final Pose pickup5controlPose = new Pose(129.65201070518754, 24.4559233626568);
    private final Pose score5Pose = new Pose(84.11984062742322, 17.12896310015135, Math.toRadians(65));


    private final Pose pickup6Pose = new Pose(132.28262287810108, 28.33822927108951, Math.toRadians(0));
    private final Pose pickup6controlPose = new Pose(103.68292957207676, 35.202832945745044);
    private final Pose score6Pose = new Pose(84.13800016396871, 17.146249511352877, Math.toRadians(90));
    private final Pose score6controlPose = new Pose(109.20213395094143, 20.55454468717135);


    private final Pose leavePose = new Pose(84.06687138626634, 28.499874971412602, Math.toRadians(90));



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

    private PathChain grabPickup2;
    //private PathChain openGate;
    private PathChain scorePickup2;
    private PathChain grabPickup3;
    private PathChain scorePickup3;
    private PathChain grabPickup4;
    private PathChain scorePickup4;
    private PathChain grabPickup5;
    private PathChain scorePickup5;
    private PathChain grabPickup6;
    private PathChain scorePickup6;
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
    }

    @Override
    public void start() {
        setState(AutoState.SHOOT);
    }

    @Override
    public void loop() {
        drive.update();
        OpModePoseStore.save(drive.getPose());

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

    @Override
    public void stop() {
        if (drive != null) {
            OpModePoseStore.save(drive.getPose());
        }
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, startPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading());

        grabPickup1 = drive.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();
        scorePickup1 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose,score1controlPose,score1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading())
                .build();
        grabPickup2 = drive.pathBuilder()
                .addPath(new BezierCurve(score1Pose, pickup2controlPose, pickup2Pose))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        /*openGate = drive.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, openGatePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), openGatePose.getHeading())
                .build();*/
        scorePickup2 = drive.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, score2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading())
                .build();
        grabPickup3 = drive.pathBuilder()
                .addPath(new BezierCurve(score2Pose,pickup3controlPose,pickup3Pose))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading())
                .build();
        scorePickup3 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, score3controlPose, score3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading())
                .build();
        grabPickup4 = drive.pathBuilder()
                .addPath(new BezierCurve(score3Pose,pickup4controlPose, pickup4Pose))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), pickup4Pose.getHeading())
                .build();
        scorePickup4 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup4Pose,score4controlPose, score4Pose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), score4Pose.getHeading())
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
                .addPath(new BezierCurve(pickup5Pose,score6controlPose, score6Pose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), score6Pose.getHeading())
                .build();
        leave = drive.pathBuilder()
                .addPath(new BezierLine(score6Pose, leavePose))
                .setLinearHeadingInterpolation(score6Pose.getHeading(), leavePose.getHeading())
                .build();

    }

    private void updateAuto() {
        switch (autoState) {

            /*case SCOREPRELOAD:
                autoAimEnabled = true;
                drive.setMaxPower(1.0);
                shooter.runClosedLoop_F(AUTO_SHOT_VELOCITY);
                drive.followPath(scorePreload);
                setState(AutoState.SHOOT);
                break;*/

            case SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = 5.0;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE1_GRAB);
                }
                break;




            case CYCLE1_GRAB:
                if (updateBurst()) {
                    //stopBurstAndShooter();
                    shooter.closeStopper();
                    //shooter.
                    startGrabPath(grabPickup1, 1.0);
                    setState(AutoState.CYCLE1_RETURN);
                }
                break;

            case CYCLE1_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup1, 0.0, 1.0);
                    shooter.runClosedLoop_F(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE1_SHOOT);
                }
                break;

            case CYCLE1_SHOOT:
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE2_GRAB);
                }
                break;



            case CYCLE2_GRAB:
                if (updateBurst()) {
                    //stopBurstAndShooter();
                    shooter.closeStopper();
                    startGrabPath(grabPickup2, 0.75);
                    setState(AutoState.CYCLE2_RETURN);
                }
                break;

            case CYCLE2_RETURN:
                if (!drive.isBusy()) {
                    startReturnPath(scorePickup2, 0.0, 1.0);
                    shooter.runClosedLoop_F(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE2_SHOOT);
                }
                break;

            case CYCLE2_SHOOT:
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE3_GRAB);
                }
                break;


            case CYCLE3_GRAB:
                if (updateBurst()) {
                    //stopBurstAndShooter();
                    shooter.closeStopper();
                    startGrabPath(grabPickup3, 1.0);
                    setState(AutoState.CYCLE3_RETURN);
                }
                break;
            case CYCLE3_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup3, 0.0, 1.0);
                    shooter.runClosedLoop_F(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE3_SHOOT);
                }
                break;
            case CYCLE3_SHOOT:
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE4_GRAB);
                }
                break;
            case CYCLE4_GRAB:
                if (updateBurst()) {
                    //stopBurstAndShooter();
                    shooter.closeStopper();
                    startGrabPath(grabPickup4, 1.0);
                    setState(AutoState.CYCLE4_RETURN);
                }
                break;


            case CYCLE4_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup4, 0.0, 1.0);
                    shooter.runClosedLoop_F(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE4_SHOOT);
                }
                break;
            case CYCLE4_SHOOT:
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE5_GRAB);
                }
                break;
            case CYCLE5_GRAB:
                if (updateBurst()) {
                    //stopBurstAndShooter();
                    shooter.closeStopper();
                    startGrabPath(grabPickup5, 1.0);
                    setState(AutoState.CYCLE5_RETURN);
                }
                break;
            case CYCLE5_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup5, 0.0, 1.0);
                    shooter.runClosedLoop_F(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE5_SHOOT);
                }
                break;
            case CYCLE5_SHOOT:
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE6_GRAB);
                }
                break;


            case CYCLE6_GRAB:
                if (updateBurst()) {
                    //stopBurstAndShooter();
                    shooter.closeStopper();
                    startGrabPath(grabPickup6, 1.0);
                    setState(AutoState.CYCLE6_RETURN);
                }
                break;
            case CYCLE6_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    startReturnPath(scorePickup6, 0.0, 1.0);
                    shooter.runClosedLoop_F(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE6_SHOOT);
                }
                break;
            case CYCLE6_SHOOT:
                if (!drive.isBusy()) {
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.STOPALL);
                }
                break;


            case STOPALL:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    startGrabPath(leave, 1.0);
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

        shooter.runClosedLoop_F(burstTargetVelocity);

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