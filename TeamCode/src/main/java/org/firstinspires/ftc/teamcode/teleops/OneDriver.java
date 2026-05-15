package org.firstinspires.ftc.teamcode.teleops;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.AutoAimTurretCommand_goal_Red;
import org.firstinspires.ftc.teamcode.commands.CenterTurretCommand_goal;
import org.firstinspires.ftc.teamcode.commands.ManualTurretCommand_goal;
import org.firstinspires.ftc.teamcode.commands.PedroDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem_goal;

@Config
@TeleOp(name = "ONE DRIVER old")
public class OneDriver extends CommandOpMode {

    private PedroDriveSubsystem drive;
    private TurretSubsystem_goal turret;
    private AutoAimTurretCommand_goal_Red autoAimCommand;

    private DcMotorEx flywheelL;
    private DcMotorEx flywheelR;
    private DcMotor intake1;
    private DcMotor intake2;
    private Servo stopper;
    private VoltageSensor batterySensor;

    //public static double TARGET_VELOCITY = 1800.0;
    public static double NORMAL_TARGET_VELOCITY = 1800.0;
    public static double FAR_TARGET_VELOCITY = 2100.0;
    //public static double CURRENT_TARGET_VELOCITY = 1800.0;
    //public static boolean FAR_SHOT_ACTIVE = false;
    public static double READY_TOLERANCE = 50.0;

    public static double SHOOTER_kV = 0.00039;
    public static double SHOOTER_kS = 0.063;
    public static double SHOOTER_kP = 0.0255;

    public static double STOPPER_CLOSED = 0.3;
    public static double STOPPER_OPEN = 0.55;

    public static double INTAKE_FEED_POWER = 1.0;
    public static double INTAKE_MANUAL_POWER = 1.0;

    public static boolean USE_VOLTAGE_COMP = true;
    public static boolean REVERSE_LEFT_SHOOTER = true;
    public static boolean REVERSE_RIGHT_SHOOTER = false;
    public static boolean REVERSE_INTAKE1 = false;
    public static boolean REVERSE_INTAKE2 = false;

    public static double STOPPER_TO_INTAKE_DELAY_MS = 80.0;

    private double initVoltage = 12.0;
    private double shooterVelocity = 0.0;
    private double shooterPower = 0.0;
    private boolean shooterReady = false;

    private boolean lastAutoAimButton = false;
    private boolean lastCenterButton = false;

    private final ElapsedTime startupTimer = new ElapsedTime();
    private static final double TURRET_STARTUP_DELAY_SEC = 0.1;

    private final ElapsedTime stopperToIntakeTimer = new ElapsedTime();
    private boolean stopperOpenedForShot = false;
    //private boolean stopperOpenedForShot = false;
    private boolean shootingUnlocked = false;

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

        flywheelL = hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheelR = hardwareMap.get(DcMotorEx.class, "shooter1");
        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        stopper = hardwareMap.servo.get("stopper");
        batterySensor = hardwareMap.voltageSensor.iterator().next();

        initVoltage = getBatteryVoltage();
        applyShooterDirections();
        stopShooterAll();

        register(drive, turret);

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

        autoAimCommand = new AutoAimTurretCommand_goal_Red(turret, drive);

        //CURRENT_TARGET_VELOCITY = NORMAL_TARGET_VELOCITY;
        //FAR_SHOT_ACTIVE = false;

        turret.setControlEnabled(false);
        startupTimer.reset();
        resetShotSequence();
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

        handleShooter();

        Pose pose = drive.getPose();

        telemetry.addData("Turret Ready", turretReady);
        telemetry.addData("Startup Time", startupTimer.seconds());
        telemetry.addData("AutoAim Held", currentAutoAimButton);

        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Pose Heading Deg", Math.toDegrees(pose.getHeading()));

        telemetry.addData("Turret Target Deg", turret.getTargetAngleDeg());
        telemetry.addData("Turret Servo Pos", turret.getCurrentServoPos());

        telemetry.addData("Shooter Velocity", shooterVelocity);
        //telemetry.addData("Shooter Target", NORMAL_TARGET_VELOCITY);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Shooter Ready", shooterReady);
        telemetry.addData("Stopper Delay Active", stopperOpenedForShot);
        telemetry.addData("Stopper Delay ms", stopperToIntakeTimer.milliseconds());
        telemetry.addData("Battery Voltage", getBatteryVoltage());

        telemetry.update();
    }

    private void handleShooter() {
        applyShooterDirections();
        //updateShooterStateValues_1();
        //updateShooterStateValues_2();

        //boolean flywheelHeld = gamepad2.right_trigger > 0.5;

        //boolean stopAllHeld = gamepad2.left_trigger > 0.5;
        boolean fullShootHeld_2 = gamepad1.left_trigger > 0.5;
        boolean intakeForwardHeld = gamepad1.right_bumper;
        boolean intakeReverseHeld = gamepad1.left_bumper;
        boolean stopperOpenHeld = gamepad1.x;
        boolean stopperCloseHeld = gamepad1.y;
        boolean fullShootHeld = gamepad1.right_trigger > 0.5;

        /*if (stopAllHeld) {
            stopShooterAll();
            resetShotSequence();
            return;
        }*/

        if (fullShootHeld_2) {
            runFlywheelsClosedLoop_2();
        } else if (fullShootHeld) {
            runFlywheelsClosedLoop_1();
        } else {
            stopFlywheels();
        }


        /*if (fullShootHeld) {
            openStopper();
            if (shooterReady) {
                if (!stopperOpenedForShot) {
                    //openStopper();
                    stopperToIntakeTimer.reset();
                    stopperOpenedForShot = true;
                } else {
                    //openStopper();
                }

                if (stopperToIntakeTimer.milliseconds() >= STOPPER_TO_INTAKE_DELAY_MS) {
                    feedIntake();
                } else {
                    stopIntake();
                }
            } else {
                closeStopper();
                stopIntake();
                resetShotSequence();
            }
            return;
        }*/

        if (fullShootHeld) {
            if (!shootingUnlocked) {
                if (!shooterReady) {
                    closeStopper();
                    stopIntake();
                    return;
                }
                shootingUnlocked = true;
                stopperOpenedForShot = true;
                openStopper();
                stopperToIntakeTimer.reset();
            }
            openStopper();
            if (stopperToIntakeTimer.milliseconds() >= STOPPER_TO_INTAKE_DELAY_MS) {
                feedIntake();
            } else {
                stopIntake();
            }

            return;
        }

        if (fullShootHeld_2) {
            if (!shootingUnlocked) {
                if (!shooterReady) {
                    closeStopper();
                    stopIntake();
                    return;
                }
                shootingUnlocked = true;
                stopperOpenedForShot = true;
                openStopper();
                stopperToIntakeTimer.reset();
            }
            openStopper();
            if (stopperToIntakeTimer.milliseconds() >= STOPPER_TO_INTAKE_DELAY_MS) {
                feedIntake();
            } else {
                stopIntake();
            }

            return;
        }

        resetShotSequence();

        if (stopperOpenHeld) {
            openStopper();
        } else if (stopperCloseHeld) {
            closeStopper();
        }

        if (intakeForwardHeld) {
            closeStopper();
            feedIntake();
        } else if (intakeReverseHeld) {
            closeStopper();
            reverseIntake();
        } else {
            closeStopper();
            stopIntake();
        }
    }

    private void updateShooterStateValues_1() {
        shooterVelocity = Math.abs(flywheelL.getVelocity());
        shooterPower = calculateShooterPower_1(shooterVelocity);
        shooterReady = shooterVelocity >= (NORMAL_TARGET_VELOCITY - READY_TOLERANCE);
    }

    private void updateShooterStateValues_2() {
        shooterVelocity = Math.abs(flywheelL.getVelocity());
        shooterPower = calculateShooterPower_2(shooterVelocity);
        shooterReady = shooterVelocity >= (FAR_TARGET_VELOCITY - READY_TOLERANCE);
    }

    private void runFlywheelsClosedLoop_1() {
        updateShooterStateValues_1();
        setFlywheelPower(shooterPower);
    }

    private void runFlywheelsClosedLoop_2() {
        updateShooterStateValues_2();
        setFlywheelPower(shooterPower);
    }

    private void stopFlywheels() {
        setFlywheelPower(0.0);
        shooterPower = 0.0;
        shooterReady = false;
    }

    private void openStopper() {
        stopper.setPosition(STOPPER_OPEN);
    }

    private void closeStopper() {
        stopper.setPosition(STOPPER_CLOSED);
    }

    private void feedIntake() {
        setIntakePower(-INTAKE_FEED_POWER);
    }

    private void reverseIntake() {
        setIntakePower(INTAKE_MANUAL_POWER);
    }

    private void stopIntake() {
        setIntakePower(0.0);
    }

    private double calculateShooterPower_1(double currentVelocity) {
        double rawPower =
                SHOOTER_kV * NORMAL_TARGET_VELOCITY +
                        SHOOTER_kS * Math.signum(NORMAL_TARGET_VELOCITY) +
                        SHOOTER_kP * (NORMAL_TARGET_VELOCITY - currentVelocity);

        if (USE_VOLTAGE_COMP) {
            double currentVoltage = getBatteryVoltage();
            rawPower *= (initVoltage / currentVoltage);
        }

        return clamp(rawPower, 0.0, 1.0);
    }

    private double calculateShooterPower_2(double currentVelocity) {
        double rawPower =
                SHOOTER_kV * FAR_TARGET_VELOCITY +
                        SHOOTER_kS * Math.signum(FAR_TARGET_VELOCITY) +
                        SHOOTER_kP * (FAR_TARGET_VELOCITY - currentVelocity);

        if (USE_VOLTAGE_COMP) {
            double currentVoltage = getBatteryVoltage();
            rawPower *= (initVoltage / currentVoltage);
        }

        return clamp(rawPower, 0.0, 1.0);
    }

    private void applyShooterDirections() {
        flywheelL.setDirection(
                REVERSE_LEFT_SHOOTER ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD
        );
        flywheelR.setDirection(
                REVERSE_RIGHT_SHOOTER ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD
        );

        intake1.setDirection(
                REVERSE_INTAKE1 ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD
        );
        intake2.setDirection(
                REVERSE_INTAKE2 ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD
        );
    }

    private void setFlywheelPower(double power) {
        flywheelL.setPower(power);
        flywheelR.setPower(power);
    }

    private void setIntakePower(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
    }

    private void stopShooterAll() {
        setFlywheelPower(0.0);
        setIntakePower(0.0);
        stopper.setPosition(STOPPER_CLOSED);
        shooterPower = 0.0;
        shooterReady = false;
    }

    private void resetShotSequence() {
        stopperOpenedForShot = false;
        shootingUnlocked = false;
        stopperToIntakeTimer.reset();
    }

    private double getBatteryVoltage() {
        double v = batterySensor.getVoltage();
        return v > 0.0 ? v : 12.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}