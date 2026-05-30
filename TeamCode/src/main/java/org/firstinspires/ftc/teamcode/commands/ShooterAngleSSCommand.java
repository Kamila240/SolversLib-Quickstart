package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Configs.AutoAimConfig_Red;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemAngleSS;

import java.util.function.BooleanSupplier;

public class ShooterAngleSSCommand extends CommandBase {

    private final ShooterSubsystemAngleSS shooter;
    private final PedroDriveSubsystem drive;

    private final BooleanSupplier normalShotHeld;
    private final BooleanSupplier normalShotHeld_k;
    private final BooleanSupplier farShotHeld;
    private final BooleanSupplier farShotHeld_k;
    private final BooleanSupplier intakeForwardHeld;
    private final BooleanSupplier intakeReverseHeld;
    private final BooleanSupplier stopperOpenHeld;
    private final BooleanSupplier stopperCloseHeld;

    private boolean stopperOpenedForShot = false;

    public ShooterAngleSSCommand(
            ShooterSubsystemAngleSS shooter,
            PedroDriveSubsystem drive,
            BooleanSupplier normalShotHeld,
            BooleanSupplier normalShotHeld_k,
            BooleanSupplier farShotHeld,
            BooleanSupplier farShotHeld_k,
            BooleanSupplier intakeForwardHeld,
            BooleanSupplier intakeReverseHeld,
            BooleanSupplier stopperOpenHeld,
            BooleanSupplier stopperCloseHeld
    ) {
        this.shooter = shooter;
        this.drive = drive;

        this.normalShotHeld = normalShotHeld;
        this.normalShotHeld_k = normalShotHeld_k;
        this.farShotHeld = farShotHeld;
        this.farShotHeld_k = farShotHeld_k;
        this.intakeForwardHeld = intakeForwardHeld;
        this.intakeReverseHeld = intakeReverseHeld;
        this.stopperOpenHeld = stopperOpenHeld;
        this.stopperCloseHeld = stopperCloseHeld;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        stopperOpenedForShot = false;

        shooter.stopAll();

        // Стартовые позиции
        shooter.setCloseAngle();
        shooter.setStopperInitial();
    }

    @Override
    public void execute() {
        boolean shotHeld =
                normalShotHeld.getAsBoolean()
                        || normalShotHeld_k.getAsBoolean()
                        || farShotHeld.getAsBoolean()
                        || farShotHeld_k.getAsBoolean();

        boolean forwardHeld = intakeForwardHeld.getAsBoolean();
        boolean reverseHeld = intakeReverseHeld.getAsBoolean();

        boolean openHeld = stopperOpenHeld.getAsBoolean();
        boolean closeHeld = stopperCloseHeld.getAsBoolean();

        if (shotHeld) {
            double distanceToGoal = getDistanceToGoal();

            // Distance выбирает только угол, velocity одна
            shooter.runDistanceShot(distanceToGoal);

            // Stopper на shooting position 0.60
            shooter.setStopperShooting();
            stopperOpenedForShot = true;
        } else {
            shooter.stopFlywheels();
            stopperOpenedForShot = false;

            // Когда не стреляем — угол вниз, stopper в начальную позицию 0.50
            shooter.setCloseAngle();

            if (openHeld) {
                shooter.setStopperShooting();
            } else if (closeHeld) {
                shooter.setStopperInitial();
            } else {
                shooter.setStopperInitial();
            }
        }

        // Intake полностью ручной
        if (reverseHeld) {
            shooter.setStopperInitial();
            shooter.reverseIntake();
        } else if (forwardHeld) {
            shooter.feedIntake();
        } else {
            shooter.stopIntake();
        }
    }

    private double getDistanceToGoal() {
        Pose pose = drive.getPose();

        double dx = AutoAimConfig_Red.GOAL_X - pose.getX();
        double dy = AutoAimConfig_Red.GOAL_Y - pose.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }

    public boolean isStopperOpenedForShot() {
        return stopperOpenedForShot;
    }

    public double getStopperDelayMs() {
        return 0.0;
    }

    @Override
    public void end(boolean interrupted) {
        stopperOpenedForShot = false;

        shooter.stopAll();

        shooter.setCloseAngle();
        shooter.setStopperInitial();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}