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
import org.firstinspires.ftc.teamcode.commands.ShooterTeleOpCommand;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem_goal;

@TeleOp(name = "ONE DRIVER")
public class ONE_DRIVER extends CommandOpMode {

    private PedroDriveSubsystem drive;
    private TurretSubsystem_goal turret;
    private ShooterSubsystem shooter;

    private AutoAimTurretCommand_goal_Red autoAimCommand;
    private ShooterTeleOpCommand shooterTeleOpCommand;

    private boolean lastAutoAimButton = false;
    private boolean lastCenterButton = false;

    private final ElapsedTime startupTimer = new ElapsedTime();
    private static final double TURRET_STARTUP_DELAY_SEC = 0.1;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        drive = new PedroDriveSubsystem(
                hardwareMap,
                telemetry,
                new Pose(134, 8, Math.toRadians(90))
        );

        turret = new TurretSubsystem_goal(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap);

        register(drive, turret, shooter);

        drive.setDefaultCommand(
                new PedroDriveCommand(
                        drive,
                        () -> gamepad1.left_stick_y,
                        () -> gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x,
                        //() -> gamepad1.right_bumper,
                        false
                )
        );

        turret.setDefaultCommand(
                new ManualTurretCommand_goal(
                        turret,
                        () -> gamepad2.right_stick_x
                )
        );

        shooterTeleOpCommand = new ShooterTeleOpCommand(
                shooter,
                () -> gamepad1.right_trigger > 0.5, // near / normal shot
                () -> gamepad2.right_trigger > 0.5,
                () -> gamepad1.left_trigger > 0.5,  // far shot
                () -> gamepad2.left_trigger > 0.5,  // far shot
                () -> gamepad1.right_bumper,        // intake forward
                () -> gamepad1.left_bumper,         // intake reverse
                () -> gamepad1.x,                   // stopper open
                () -> gamepad1.y                    // stopper close
        );

        shooter.setDefaultCommand(shooterTeleOpCommand);

        autoAimCommand = new AutoAimTurretCommand_goal_Red(turret, drive);

        turret.setControlEnabled(false);
        startupTimer.reset();
    }

    @Override
    public void run() {
        boolean turretReady = startupTimer.seconds() > TURRET_STARTUP_DELAY_SEC;
        turret.setControlEnabled(turretReady);

        boolean currentAutoAimButton = gamepad1.b;
        boolean currentCenterButton = gamepad1.a;

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

        telemetry.addData("Turret Ready", turretReady);
        telemetry.addData("Startup Time", startupTimer.seconds());
        telemetry.addData("AutoAim Held", currentAutoAimButton);

        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Pose Heading Deg", Math.toDegrees(pose.getHeading()));

        telemetry.addData("Turret Target Deg", turret.getTargetAngleDeg());
        telemetry.addData("Turret Servo Pos", turret.getCurrentServoPos());

        telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
        telemetry.addData("Shooter Target", shooter.getActiveTargetVelocity());
        telemetry.addData("Shooter Power", shooter.getShooterPower());
        telemetry.addData("Shooter Ready", shooter.isShooterReady());

        telemetry.addData("Stopper Delay Active", shooterTeleOpCommand.isStopperOpenedForShot());
        telemetry.addData("Stopper Delay ms", shooterTeleOpCommand.getStopperDelayMs());
        telemetry.addData("Stopper Pos", shooter.getStopperPosition());

        telemetry.addData("Battery Voltage", shooter.getBatteryVoltage());

        telemetry.update();
    }
}