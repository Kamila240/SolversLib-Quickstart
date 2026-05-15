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

@Autonomous(name = "Red_Gate_test", group = "Autonomous")
public class Red_Gate_test extends OpMode {

    private PedroDriveSubsystem drive;
    private ShooterSubsystem shooter;
    private TurretSubsystem_goal turret;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime burstTimer = new ElapsedTime();

    private boolean autoAimEnabled = true;

    private boolean arrivalLatched = false;


    private final ElapsedTime shooterStableTimer = new ElapsedTime();
    private boolean shooterStableStarted = false;

    private static final double SHOOTER_STABLE_MS = 100.0;

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
        CYCLE2_GATE,
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
        //CYCLE5_GATE,
        CYCLE5_RETURN,
        CYCLE5_SHOOT,

        CYCLE6_GRAB,
        CYCLE6_RETURN,
        CYCLE6_SHOOT,

        STOPALL,

        DONE
    }

    private AutoState autoState = AutoState.SCOREPRELOAD;

    private static final double AUTO_SHOT_VELOCITY = 1300.0;

    private static final long INTAKE_TIME_LONG_MS = 850;
    private static final long INTAKE_TIME_SHORT_MS = 750;

    private static final double STOPPER_TO_INTAKE_DELAY_MS = 20.0;
    private static final double FEED_TIME_MS = 450.0;
    private static final double RECOVER_TIME_MS = 5.0;

    private final Pose startPose = new Pose(124.9867601246106, 112.69236760124609, Math.toRadians(0));
    private final Pose scorePose = new Pose(91.32510706775544, 82.21230529595016, Math.toRadians(-30));

    private final Pose pickup1Pose = new Pose(119.68456171087249, 57.67703632963949, Math.toRadians(0));
    private final Pose pickup1controlPose = new Pose(101.04086171435043, 52.290491103774066);
    private final Pose score1Pose = new Pose(94.42187174889776, 79.34019299256126, Math.toRadians(-45));

    private final Pose pickup2Pose = new Pose(118.32324570496132, 62.01188302960525, Math.toRadians(20));
    private final Pose pickup2controlPose = new Pose(109.87687728795063, 65.4686693359521);
    private final Pose gate1Pose = new Pose(128.96947040498449, 53.04497350288997, Math.toRadians(35));
    private final Pose gate1controlPose = new Pose(123.50858459400757, 51.24422835862195);
    private final Pose score2Pose = new Pose(94.553738317757, 79.34579439252336, Math.toRadians(-45));
    private final Pose score2controlPose = new Pose(108.82593449763738, 63.25040148848227);

    private final Pose pickup3Pose = new Pose(118.32307314346393, 62.09667387497936, Math.toRadians(20));
    private final Pose pickup3controlPose = new Pose(109.85377810099313, 65.62126725085099);
    private final Pose gate2Pose = new Pose(128.15004946634303, 53.04821957874208, Math.toRadians(35));
    private final Pose gate2controlPose = new Pose(123.59170149181935, 51.17252460848066);
    private final Pose score3Pose = new Pose(94.62797869151103, 79.29506806641889, Math.toRadians(-45));
    private final Pose score3controlPose = new Pose(108.74415426584291, 63.15528868239358);

    private final Pose pickup4Pose = new Pose(118.42468130652938, 62.05790908201639, Math.toRadians(20));
    private final Pose pickup4controlPose = new Pose(109.87687728795063, 65.4686693359521);
    private final Pose gate3Pose = new Pose(128.05536087154988, 53.095052652132246, Math.toRadians(35));
    private final Pose gate3controlPose = new Pose(123.66447591770005, 51.20926902906808);
    private final Pose score4Pose = new Pose(93.66780648807044, 80.80124177901645, Math.toRadians(0));
    private final Pose score4controlPose = new Pose(106.34328149912481, 60.225795190652214);

    private final Pose pickup5Pose = new Pose(116.17127970106696, 81.07518528001425, Math.toRadians(0));
    //private final Pose pickup5controlPose = new Pose(109.87687728795063, 65.4686693359521);
    //private final Pose gate4Pose = new Pose(128.6509626865094, 50.92225011864848, Math.toRadians(25));
    //private final Pose gate4controlPose = new Pose(123.50345933592222, 51.29572701026395);
    private final Pose score5Pose = new Pose(93.63922678390554, 80.87268952460639, Math.toRadians(-45));
    //private final Pose score5controlPose = new Pose(108.82996672103761, 63.27240712355774);

    private final Pose pickup6Pose = new Pose(117.92486254735984, 34.73189116564665, Math.toRadians(0));
    private final Pose pickup6controlPose1 = new Pose(94.18625027310932, 26.93741495571842);
    private final Pose score6Pose = new Pose(81.14554235391982, 98.56410076828332, Math.toRadians(-60));


    private Path scorePreload;

    private PathChain grabPickup1;
    private PathChain scorePickup1;

    private PathChain gate1;
    private PathChain gate2;
    private PathChain gate3;
    //private PathChain gate4;

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

        //--------------//
        grabPickup1 = drive.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup1controlPose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = drive.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, score1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading())
                .build();


        //---------------//
        grabPickup2 = drive.pathBuilder()
                .addPath(new BezierCurve(score1Pose, pickup2controlPose, pickup2Pose))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        gate1 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose,gate1controlPose, gate1Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), gate1Pose.getHeading())
                .build();

        scorePickup2 = drive.pathBuilder()
                .addPath(new BezierCurve(gate1Pose, score2controlPose,score2Pose))
                .setLinearHeadingInterpolation(gate1Pose.getHeading(), score2Pose.getHeading())
                .build();


        //----------------//
        grabPickup3 = drive.pathBuilder()
                .addPath(new BezierCurve(score2Pose,pickup3controlPose,pickup3Pose))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        gate2 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, gate2controlPose,gate2Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), gate2Pose.getHeading())
                .build();

        scorePickup3 = drive.pathBuilder()
                .addPath(new BezierCurve(gate2Pose,score3controlPose, score3Pose))
                .setLinearHeadingInterpolation(gate2Pose.getHeading(), score3Pose.getHeading())
                .build();

        //---------------//
        grabPickup4 = drive.pathBuilder()
                .addPath(new BezierCurve(score3Pose,pickup4controlPose,pickup4Pose))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), pickup4Pose.getHeading())
                .build();

        gate3 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup4Pose, gate3controlPose,gate3Pose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), gate3Pose.getHeading())
                .build();

        scorePickup4 = drive.pathBuilder()
                .addPath(new BezierCurve(gate3Pose,score4controlPose, score4Pose))
                .setLinearHeadingInterpolation(gate3Pose.getHeading(), score4Pose.getHeading())
                .build();

        //------------------//
        grabPickup5 = drive.pathBuilder()
                .addPath(new BezierLine(score4Pose,pickup5Pose))
                .setLinearHeadingInterpolation(score4Pose.getHeading(), pickup5Pose.getHeading())
                .build();

        /*gate4 = drive.pathBuilder()
                .addPath(new BezierCurve(pickup5Pose, gate4controlPose,gate4Pose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), gate4Pose.getHeading())
                .build();*/

        scorePickup5 = drive.pathBuilder()
                .addPath(new BezierLine(pickup5Pose,score5Pose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), score5Pose.getHeading())
                .build();

        //--------------//
        grabPickup6 = drive.pathBuilder()
                .addPath(new BezierCurve(score5Pose, pickup6controlPose1, pickup6Pose))
                .setLinearHeadingInterpolation(score5Pose.getHeading(), pickup6Pose.getHeading())
                .build();

        scorePickup6 = drive.pathBuilder()
                .addPath(new BezierLine(pickup6Pose, score6Pose))
                .setLinearHeadingInterpolation(pickup6Pose.getHeading(), score6Pose.getHeading())
                .build();

    }

    private void updateAuto() {
        switch (autoState) {

            case SCOREPRELOAD:
                drive.setMaxPower(1.0);
                shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                TurretConfig_goal.OFFSET = -5.0;
                drive.followPath(scorePreload);
                setState(AutoState.SHOOT);
                break;

            case SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -5.0;
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
                if (handleCollectTotal(INTAKE_TIME_SHORT_MS)) {
                    TurretConfig_goal.OFFSET = -5.0;
                    shooter.feedIntake();
                    startReturnPath(scorePickup1, -5.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    shooter.stopIntake();
                    setState(AutoState.CYCLE1_SHOOT);
                }
                break;

            case CYCLE1_SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -5.0;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE2_GRAB);
                }
                break;



            case CYCLE2_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    drive.setMaxPower(0.85);
                    drive.followPath(grabPickup2, true);
                    shooter.feedIntake();
                    setState(AutoState.CYCLE2_GATE);
                }
                break;

            case CYCLE2_GATE:
                if(!drive.isBusy()){
                    /*startGrabPath(gate1, 0.85);
                    setState(AutoState.CYCLE2_RETURN);*/
                    if (waitMs(300)){
                        //shooter.feedIntake();
                        startGrabPath(gate1, 0.85);
                        shooter.feedIntake();
                        setState(AutoState.CYCLE2_RETURN);}
                }
                break;
            case CYCLE2_RETURN:
                if (handleCollectExtraAfterDriveDone(INTAKE_TIME_LONG_MS)) {
                    TurretConfig_goal.OFFSET = -5.0;
                    shooter.feedIntake();
                    //shooter.stopIntake();
                    startReturnPath(scorePickup2, -5.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    shooter.stopIntake();
                    setState(AutoState.CYCLE2_SHOOT);
                }
                break;
            case CYCLE2_SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -5.0;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE3_GRAB);
                }
                break;



            case CYCLE3_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    drive.setMaxPower(0.85);
                    drive.followPath(grabPickup3, true);
                    shooter.feedIntake();
                    setState(AutoState.CYCLE3_GATE);
                }
                break;

            case CYCLE3_GATE:
                if(!drive.isBusy()){
                    /*startGrabPath(gate2, 0.85);
                    setState(AutoState.CYCLE3_RETURN);*/
                    if (waitMs(300)){
                        //shooter.feedIntake();
                        startGrabPath(gate2, 0.85);
                        shooter.feedIntake();
                        setState(AutoState.CYCLE3_RETURN);}
                }
                break;
            case CYCLE3_RETURN:
                if (handleCollectExtraAfterDriveDone(INTAKE_TIME_LONG_MS)) {
                    TurretConfig_goal.OFFSET = -5.0;
                    //shooter.stopIntake();
                    shooter.feedIntake();
                    startReturnPath(scorePickup3, -5.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    shooter.stopIntake();
                    setState(AutoState.CYCLE3_SHOOT);
                }
                break;
            case CYCLE3_SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -5.0;
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
                    /*startGrabPath(gate3, 0.85);
                    setState(AutoState.CYCLE4_RETURN);*/
                    if (waitMs(300)){
                        //shooter.feedIntake();
                        startGrabPath(gate3, 0.85);
                        shooter.feedIntake();
                        setState(AutoState.CYCLE4_RETURN);}
                }
                break;
            case CYCLE4_RETURN:
                if (handleCollectExtraAfterDriveDone(INTAKE_TIME_LONG_MS)) {
                    TurretConfig_goal.OFFSET = -5.0;
                    shooter.feedIntake();
                    //shooter.stopIntake();
                    startReturnPath(scorePickup4, -5.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    shooter.stopIntake();
                    setState(AutoState.CYCLE4_SHOOT);
                }
                break;
            case CYCLE4_SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -5.0;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE5_GRAB);
                }
                break;




            case CYCLE5_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    startGrabPath(grabPickup5, 1.00);
                    setState(AutoState.CYCLE5_RETURN);
                }
                break;

            case CYCLE5_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    TurretConfig_goal.OFFSET = -5.0;
                    //shooter.stopIntake();
                    shooter.feedIntake();
                    startReturnPath(scorePickup5, -5.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    shooter.stopIntake();
                    setState(AutoState.CYCLE5_SHOOT);
                }
                break;

            case CYCLE5_SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -5.0;
                    startBurst(1, AUTO_SHOT_VELOCITY);
                    setState(AutoState.CYCLE6_GRAB);
                }
                break;



            case CYCLE6_GRAB:
                if (updateBurst()) {
                    stopBurstAndShooter();
                    startGrabPath(grabPickup6, 1.00);
                    setState(AutoState.CYCLE6_RETURN);
                }
                break;

            case CYCLE6_RETURN:
                if (handleCollectTotal(INTAKE_TIME_LONG_MS)) {
                    TurretConfig_goal.OFFSET = -2.0;
                    //shooter.stopIntake();
                    shooter.feedIntake();
                    startReturnPath(scorePickup6, -5.0, 1.0);
                    shooter.runClosedLoop_N(AUTO_SHOT_VELOCITY);
                    shooter.stopIntake();
                    setState(AutoState.CYCLE6_SHOOT);
                }
                break;

            case CYCLE6_SHOOT:
                if (!drive.isBusy()) {
                    TurretConfig_goal.OFFSET = -7.0;
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

        shooterStableStarted = false;
        shooterStableTimer.reset();
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
                if (shooterReadyStable(burstTargetVelocity)) {
                    shooter.openStopper();
                    burstTimer.reset();
                    burstState = BurstState.OPEN_DELAY;
                }
                break;

            case OPEN_DELAY:
                if (!shooter.isReadyFor(burstTargetVelocity)) {
                    shooter.closeStopper();
                    shooter.stopIntake();

                    shooterStableStarted = false;
                    shooterStableTimer.reset();

                    burstState = BurstState.SPINUP;
                    break;
                }

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
                        shooterStableStarted = false;
                        shooterStableTimer.reset();
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

    private boolean shooterReadyStable(double targetVelocity) {
        if (shooter.isReadyFor(targetVelocity)) {
            if (!shooterStableStarted) {
                shooterStableStarted = true;
                shooterStableTimer.reset();
                return false;
            }

            return shooterStableTimer.milliseconds() >= SHOOTER_STABLE_MS;
        }

        shooterStableStarted = false;
        shooterStableTimer.reset();
        return false;
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