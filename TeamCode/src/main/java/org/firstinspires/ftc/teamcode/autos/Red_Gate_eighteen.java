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

@Autonomous(name = "Red_Gate_18", group = "Autonomous")
public class Red_Gate_eighteen extends OpMode {

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
        CYCLE3_GATE,
        CYCLE3_RETURN,
        CYCLE3_SHOOT,

        CYCLE4_GRAB,
        CYCLE4_GATE,
        CYCLE4_RETURN,
        CYCLE4_SHOOT,

        CYCLE5_GRAB,
        CYCLE5_RETURN,
        CYCLE5_SHOOT,

        /*CYCLE6_GRAB,
        CYCLE6_RETURN,
        CYCLE6_SHOOT,*/

        STOPALL,

        DONE
    }

    private AutoState autoState = AutoState.SCOREPRELOAD;

    private static final double AUTO_SHOT_VELOCITY = 1500.0;

    private static final long INTAKE_TIME_LONG_MS = 1000;
    private static final long INTAKE_TIME_SHORT_MS = 800;

    private static final double STOPPER_TO_INTAKE_DELAY_MS = 30.0;
    private static final double FEED_TIME_MS = 950.0;
    private static final double RECOVER_TIME_MS = 10.0;

    private final Pose startPose = new Pose(124.9867601246106, 112.69236760124609, Math.toRadians(0));
    private final Pose scorePose = new Pose(96.3944217095, 86.47714584743947, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(116.64034000026861, 80.4903900994342, Math.toRadians(0));
    private final Pose pickup1controlPose = new Pose(100.72507788161992, 77.36214953271029);
    private final Pose score1Pose = new Pose(93.02566639072563, 82.43821379068126, Math.toRadians(-30));
    private final Pose pickup2Pose = new Pose(119.68456171087249, 57.67703632963949, Math.toRadians(0));
    private final Pose pickup2controlPose = new Pose(101.04086171435043, 52.290491103774066);
    private final Pose openGatePose = new Pose(123.92022030145199, 64.98237987296167, Math.toRadians(0));
    //private final Pose openGatecontrolPose = new Pose(116.51267137999399, 67.26428754055293);
    private final Pose score2Pose = new Pose(92.87903685793204, 82.42586277449269, Math.toRadians(-45));
    private final Pose pickup3Pose = new Pose(120.40406451539373, 61.95750190594276, Math.toRadians(20));
    private final Pose pickup3controlPose = new Pose(109.85377810099313, 65.47750788538909);
    private final Pose gate1Pose = new Pose(128.6, 51.04497350288995, Math.toRadians(25));
    private final Pose gate1controlPose = new Pose(123.50858459400757, 51.24422835862195);
    private final Pose score3Pose = new Pose(93.02240918246646, 82.33744941176933, Math.toRadians(-45));
    private final Pose score3controlPose = new Pose(108.82593449763738, 63.25040148848227);
    private final Pose pickup4Pose = new Pose(120.32324570496132, 62.01188302960525, Math.toRadians(20));
    private final Pose pickup4controlPose = new Pose(109.87687728795063, 65.4686693359521);
    private final Pose gate2Pose = new Pose(128.6509626865094, 50.92225011864848, Math.toRadians(25));
    private final Pose gate2controlPose = new Pose(123.50345933592222, 51.29572701026395);
    private final Pose score4Pose = new Pose(92.97626047519203, 82.42240836522713, Math.toRadians(-45));
    private final Pose score4controlPose = new Pose(108.82996672103761, 63.27240712355774);
    private final Pose pickup5Pose = new Pose(126.30185559846744, 32.48767472616137, Math.toRadians(25));
    private final Pose pickup5controlPose1 = new Pose(91.46806853582552, 20.93847352024924);
    private final Pose score5Pose = new Pose(77.62258234265508, 99.82890011629802, Math.toRadians(-45));


    private Path scorePreload;

    private PathChain grabPickup1;
    private PathChain scorePickup1;
    private PathChain gate1;
    private PathChain gate2;
    private PathChain grabPickup2;
    private PathChain openGate;
    private PathChain scorePickup2;
    private PathChain grabPickup3;
    private PathChain scorePickup3;
    private PathChain grabPickup4;
    private PathChain scorePickup4;
    private PathChain grabPickup5;
    private PathChain scorePickup5;
    //private PathChain grabPickup6;
    //private PathChain scorePickup6;

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
        //autoAimEnabled = true;
        arrivalLatched = false;
    }

    @Override
    public void start() {
        setState(AutoState.SCOREPRELOAD);
    }

    @Override
    public void loop() {
        drive.update();
        turret.aimAtFieldPoint(
                drive.getPose(),
                AutoAimConfig_Red.GOAL_X,
                AutoAimConfig_Red.GOAL_Y
        );

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
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading())
                .build();

        grabPickup2 = drive.pathBuilder()
                .addPath(new BezierCurve(score1Pose, pickup2controlPose, pickup2Pose))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup2Pose.getHeading())
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
                .addPath(new BezierCurve(score2Pose,pickup3controlPose,pickup3Pose))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        gate1 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, gate1controlPose,gate1Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), gate1Pose.getHeading())
                .build();

        scorePickup3 = drive.pathBuilder()
                .addPath(new BezierCurve(gate1Pose,score3controlPose, score3Pose))
                .setLinearHeadingInterpolation(gate1Pose.getHeading(), score3Pose.getHeading())
                .build();

        grabPickup4 = drive.pathBuilder()
                .addPath(new BezierCurve(score3Pose,pickup4controlPose,pickup4Pose))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), pickup4Pose.getHeading())
                .build();

        gate2 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup4Pose, gate2controlPose,gate2Pose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), gate2Pose.getHeading())
                .build();

        scorePickup4 = drive.pathBuilder()
                .addPath(new BezierCurve(gate2Pose,score4controlPose, score4Pose))
                .setLinearHeadingInterpolation(gate2Pose.getHeading(), score4Pose.getHeading())
                .build();

        grabPickup5 = drive.pathBuilder()
                .addPath(new BezierCurve(score4Pose, pickup5controlPose1,pickup5Pose))
                .setLinearHeadingInterpolation(score4Pose.getHeading(), pickup5Pose.getHeading())
                .build();

        scorePickup5 = drive.pathBuilder()
                .addPath(new BezierLine(pickup5Pose, score5Pose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), score5Pose.getHeading())
                .build();

        /*grabPickup6 = drive.pathBuilder()
                .addPath(new BezierCurve(score5Pose, pickup6controlPose, pickup6Pose))
                .setLinearHeadingInterpolation(score5Pose.getHeading(), pickup6Pose.getHeading())
                .build();

        scorePickup6 = drive.pathBuilder()
                .addPath(new BezierLine(pickup5Pose, score6Pose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), score6Pose.getHeading())
                .build();*/
    }

    private void updateAuto() {
        switch (autoState) {

            case SCOREPRELOAD:
                //autoAimEnabled = true;
                //TurretConfig_goal.OFFSET = -2.0;
                drive.setMaxPower(1.0);
                shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                drive.followPath(scorePreload);
                setState(AutoState.SHOOT);
                break;

            case SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    //autoAimEnabled = true;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE1_GRAB);
                }
                break;


            case CYCLE1_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    startGrabPath(grabPickup1, 0.9);
                    setState(AutoState.CYCLE1_RETURN);
                }
                break;

            case CYCLE1_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    TurretConfig_goal.OFFSET = -2.0;
                    startReturnPath(scorePickup1, 0.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    //autoAimEnabled = true;
                    setState(AutoState.CYCLE1_SHOOT);
                }
                break;

            case CYCLE1_SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -2.0;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE2_GRAB);
                }
                break;

            case CYCLE2_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    startGrabPath(grabPickup2, 1.0);
                    setState(AutoState.CYCLE2_OPENGATE);
                }
                break;

            case CYCLE2_OPENGATE:
                if (handleCollectExtraAfterDriveDone(INTAKE_TIME_SHORT_MS - 500)) {
                    drive.setMaxPower(1.0);
                    drive.followPath(openGate, true);
                    setState(AutoState.CYCLE2_RETURN);
                }
                break;

            case CYCLE2_RETURN:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -2.0;
                    shooter.stopIntake();
                    startReturnPath(scorePickup2, 0.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE2_SHOOT);
                }
                break;

            case CYCLE2_SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -2.0;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE3_GRAB);
                }
                break;



            /*if (waitMs(300)) {
                shooter.feedIntake();
                startGrabPath(gate1, 0.85);
                setState(AutoState.CYCLE3_RETURN);
            }*/

            case CYCLE3_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    drive.setMaxPower(0.85);
                    drive.followPath(grabPickup3, true);
                    //shooter.feedIntake();
                    setState(AutoState.CYCLE3_GATE);
                }
                break;

            case CYCLE3_GATE:
                if(!drive.isBusy()){
                    if (waitMs(300)){
                    shooter.feedIntake();
                    startGrabPath(gate1, 0.85);
                    setState(AutoState.CYCLE3_RETURN);}
                }
                break;
            case CYCLE3_RETURN:
                if (handleCollectExtraAfterDriveDone(INTAKE_TIME_LONG_MS + 100)) {
                    shooter.stopIntake();
                    TurretConfig_goal.OFFSET = -2.0;
                    startReturnPath(scorePickup3, 0.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE3_SHOOT);
                }
                break;
            case CYCLE3_SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -2.0;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE4_GRAB);
                }
                break;



            case CYCLE4_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    drive.setMaxPower(0.85);
                    drive.followPath(grabPickup4, true);
                    setState(AutoState.CYCLE4_GATE);
                }
                break;

            case CYCLE4_GATE:
                if(!drive.isBusy()){
                    if (waitMs(300)){
                    shooter.feedIntake();
                    startGrabPath(gate2, 0.85);
                    setState(AutoState.CYCLE4_RETURN);}
                }
                break;
            case CYCLE4_RETURN:
                /*if(!drive.isBusy()) {
                startReturnPath(scorePickup3, 0.0, 1.0);
                shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                setState(AutoState.CYCLE3_SHOOT);
                }
                break;*/
                if (handleCollectExtraAfterDriveDone(INTAKE_TIME_LONG_MS + 100)) {
                    shooter.stopIntake();
                    TurretConfig_goal.OFFSET = -2.0;
                    /*drive.setMaxPower(0.75);
                    drive.followPath(openGate, true);
                    setState(AutoState.CYCLE2_RETURN);*/
                    startReturnPath(scorePickup4, 0.0, 1.0);
                    //autoAimEnabled = true;
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE4_SHOOT);
                }
                break;
            case CYCLE4_SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -2.0;
                    //autoAimEnabled = true;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE5_GRAB);
                }
                break;



            case CYCLE5_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    startGrabPath(grabPickup5, 0.95);
                    setState(AutoState.CYCLE5_RETURN);
                }
                break;

            case CYCLE5_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS + 200)) {
                    shooter.stopIntake();
                    startReturnPath(scorePickup5, 0.0, 1.0);
                    //autoAimEnabled = true;
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE5_SHOOT);
                }
                break;

            case CYCLE5_SHOOT:
                //shooter.runClosedLoop(AUTO_SHOT_VELOCITY);
                if (!drive.isBusy()) {
                    //autoAimEnabled = true;
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
                //autoAimEnabled = false;
                break;
        }
    }

    private void setState(AutoState newState) {
        autoState = newState;
        stateTimer.reset();
        arrivalLatched = false;
    }

    private void startGrabPath(PathChain path, double maxPower) {
        //autoAimEnabled = false;

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

        //autoAimEnabled = true;
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

    private boolean waitMs(long ms) {
        return stateTimer.milliseconds() >= ms;
    }


    private void stopBurstAndShooter() {
        burstStarted = false;
        burstState = BurstState.IDLE;

        shooter.stopIntake();
        shooter.closeStopper();
        shooter.stopFlywheels();
    }
}