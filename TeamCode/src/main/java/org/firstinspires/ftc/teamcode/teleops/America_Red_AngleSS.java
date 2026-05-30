package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.AutoAimTurretCommand_goal_Red;
import org.firstinspires.ftc.teamcode.commands.CenterTurretCommand_goal;
import org.firstinspires.ftc.teamcode.commands.ManualTurretCommand_goal;
import org.firstinspires.ftc.teamcode.commands.PedroDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterAngleSSCommand;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemAngleSS;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem_goal;

@TeleOp(name = "America RED Angle SS")
public class America_Red_AngleSS extends CommandOpMode {

    private PedroDriveSubsystem drive;
    private TurretSubsystem_goal turret;
    private ShooterSubsystemAngleSS shooter;

    private AutoAimTurretCommand_goal_Red autoAimCommand;
    private ShooterAngleSSCommand shooterTeleOpCommand;

    private boolean lastAutoAimButton = false;
    private boolean lastCenterButton = false;
    private boolean lastPoseResetButton = false;

    private final ElapsedTime startupTimer = new ElapsedTime();
    private static final double TURRET_STARTUP_DELAY_SEC = 0.1;

    private static final Pose RESET_POSE = new Pose(
            127.19080996884739,
            77.20716510903429,
            Math.toRadians(0)
    );

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        drive = new PedroDriveSubsystem(
                hardwareMap,
                telemetry,
                new Pose(126.08443868626176, 76.55713451857179, Math.toRadians(0))
        );

        turret = new TurretSubsystem_goal(hardwareMap, telemetry);
        shooter = new ShooterSubsystemAngleSS(hardwareMap);

        register(drive, turret, shooter);

        drive.setDefaultCommand(
                new PedroDriveCommand(
                        drive,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x,
                        true
                )
        );

        turret.setDefaultCommand(
                new ManualTurretCommand_goal(
                        turret,
                        () -> {
                            // D-PAD управление туррелью
                            if (gamepad1.dpad_left) {
                                return -1.0; // влево
                            }

                            if (gamepad1.dpad_right) {
                                return 1.0; // вправо / обратная сторона
                            }

                            // Если D-PAD не нажимается, работает правый стик
                            return gamepad2.right_stick_x;
                        }
                )
        );

        shooterTeleOpCommand = new ShooterAngleSSCommand(
                shooter,
                drive,
                () -> gamepad2.right_trigger > 0.5,
                () -> gamepad1.right_trigger > 0.5,
                () -> gamepad2.left_trigger > 0.5,
                () -> gamepad1.left_trigger > 0.5,
                () -> gamepad2.right_bumper,
                () -> gamepad2.left_bumper,
                () -> false,
                () -> false
        );

        shooter.setDefaultCommand(shooterTeleOpCommand);

        autoAimCommand = new AutoAimTurretCommand_goal_Red(turret, drive);

        turret.setControlEnabled(false);
        startupTimer.reset();

        // На INIT сразу:
        // угол вниз + stopper initial
        shooter.setCloseAngle();
        shooter.setStopperInitial();
    }

    @Override
    public void run() {
        boolean turretReady = startupTimer.seconds() > TURRET_STARTUP_DELAY_SEC;
        turret.setControlEnabled(turretReady);

        boolean currentAutoAimButton = gamepad1.b;
        boolean currentCenterButton = gamepad1.a;

        // Reset pose только через комбинацию
        boolean currentPoseResetButton =  gamepad1.y;

        if (currentPoseResetButton && !lastPoseResetButton) {
            drive.setPose(RESET_POSE);
        }
        lastPoseResetButton = currentPoseResetButton;

        if (turretReady) {
            if (currentAutoAimButton && !lastAutoAimButton) {
                CommandScheduler.getInstance().schedule(autoAimCommand);
            }

            if (!currentAutoAimButton && lastAutoAimButton) {
                CommandScheduler.getInstance().cancel(autoAimCommand);
            }

            if (currentCenterButton && !lastCenterButton) {
                CommandScheduler.getInstance().schedule(new CenterTurretCommand_goal(turret));
            }
        } else {
            CommandScheduler.getInstance().cancel(autoAimCommand);
        }

        lastAutoAimButton = currentAutoAimButton;
        lastCenterButton = currentCenterButton;

        super.run();

        Pose pose = drive.getPose();

        telemetry.addData("--- DRIVE / TURRET ---", "");
        telemetry.addData("Turret Ready", turretReady);
        telemetry.addData("AutoAim Held", currentAutoAimButton);

        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Pose Heading Deg", Math.toDegrees(pose.getHeading()));

        telemetry.addData("Turret Target Deg", turret.getTargetAngleDeg());
        telemetry.addData("Turret Servo Pos", turret.getCurrentServoPos());

        telemetry.addData("--- SHOOTER ANGLE ---", "");
        telemetry.addData("Shooter Zone", shooter.getActiveZone());
        telemetry.addData("Distance To Goal", shooter.getActiveDistanceToGoal());

        telemetry.addData("Angle Main Pos", shooter.getShooterAnglePosition());
        telemetry.addData("Angle Left Pos", shooter.getLeftAnglePosition());
        telemetry.addData("Angle Right Pos", shooter.getRightAnglePosition());

        telemetry.addData("Optimal Target Velocity", ShooterSubsystemAngleSS.OPTIMAL_TARGET_VELOCITY);
        telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
        telemetry.addData("Shooter Target", shooter.getActiveTargetVelocity());
        telemetry.addData("Shooter Power", shooter.getShooterPower());
        telemetry.addData("Shooter Ready", shooter.isShooterReady());

        telemetry.addData("Stopper Opened For Shot", shooterTeleOpCommand.isStopperOpenedForShot());
        telemetry.addData("Stopper Pos AngleSS", shooter.getAngleSSStopperPosition());

        telemetry.addData("Battery Voltage", shooter.getBatteryVoltage());

        telemetry.update();
    }
}