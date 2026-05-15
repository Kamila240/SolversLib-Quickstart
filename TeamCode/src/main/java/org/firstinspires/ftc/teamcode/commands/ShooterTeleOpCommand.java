package org.firstinspires.ftc.teamcode.commands;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class ShooterTeleOpCommand extends CommandBase {

    public static double STOPPER_TO_INTAKE_DELAY_MS = 120.0;

    private final ShooterSubsystem shooter;

    private final BooleanSupplier normalShotHeld;
    private final BooleanSupplier normalShotHeld_k;
    private final BooleanSupplier farShotHeld;
    private final BooleanSupplier farShotHeld_k;
    private final BooleanSupplier intakeForwardHeld;
    private final BooleanSupplier intakeReverseHeld;
    private final BooleanSupplier stopperOpenHeld;
    private final BooleanSupplier stopperCloseHeld;

    private final ElapsedTime stopperToIntakeTimer = new ElapsedTime();

    private boolean stopperOpenedForShot = false;
    private boolean shootingUnlocked = false;

    public ShooterTeleOpCommand(
            ShooterSubsystem shooter,
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
        resetShotSequence();
        shooter.stopAll();
    }

    @Override
    public void execute() {
        boolean nearHeld = normalShotHeld.getAsBoolean();
        boolean nearHeld_k = normalShotHeld_k.getAsBoolean();
        boolean farHeld = farShotHeld.getAsBoolean();
        boolean farHeld_k = farShotHeld_k.getAsBoolean();
        boolean forwardHeld = intakeForwardHeld.getAsBoolean();
        boolean reverseHeld = intakeReverseHeld.getAsBoolean();
        boolean openHeld = stopperOpenHeld.getAsBoolean();
        boolean closeHeld = stopperCloseHeld.getAsBoolean();

        if (farHeld) {
            shooter.runClosedLoop_F(ShooterSubsystem.FAR_TARGET_VELOCITY);
        }
        else if (farHeld_k){
            shooter.runClosedLoop_F(ShooterSubsystem.FAR_TARGET_VELOCITY);
        }
        else if (nearHeld) {
            shooter.runClosedLoop_N(ShooterSubsystem.NORMAL_TARGET_VELOCITY);
        } else if (nearHeld_k) {
            shooter.runClosedLoop_N(ShooterSubsystem.NORMAL_TARGET_VELOCITY);
        } else {
            shooter.stopFlywheels();
        }

        if (nearHeld) {
            runShootSequence(ShooterSubsystem.NORMAL_TARGET_VELOCITY);
            return;
        }

        if (nearHeld_k) {
            runShootSequence_1(ShooterSubsystem.NORMAL_TARGET_VELOCITY);
            return;
        }

        if (farHeld) {
            runShootSequence(ShooterSubsystem.FAR_TARGET_VELOCITY);
            return;
        }

        if (farHeld_k) {
            runShootSequence_1(ShooterSubsystem.FAR_TARGET_VELOCITY);
            return;
        }

        resetShotSequence();

        if (openHeld) {
            shooter.openStopper();
        } else if (closeHeld) {
            shooter.closeStopper();
        }

        if (forwardHeld) {
            shooter.closeStopper();
            shooter.feedIntake();
        } else if (reverseHeld) {
            shooter.closeStopper();
            shooter.reverseIntake();
        } else {
            shooter.closeStopper();
            shooter.stopIntake();
        }
    }

    private void runShootSequence(double targetVelocity) {
        if (!shootingUnlocked) {
            if (!shooter.isReadyFor(targetVelocity)) {
                shooter.closeStopper();
                shooter.stopIntake();
                return;
            }

            shootingUnlocked = true;
            stopperOpenedForShot = true;
            shooter.openStopper();
            stopperToIntakeTimer.reset();
        }

        shooter.openStopper();

        if (stopperToIntakeTimer.milliseconds() >= STOPPER_TO_INTAKE_DELAY_MS) {
            shooter.feedIntake();
        } else {
            shooter.stopIntake();
        }
    }

    private void runShootSequence_1(double targetVelocity) {
        if (!shootingUnlocked) {
            if (!shooter.isReadyFor(targetVelocity)) {
                shooter.closeStopper();
                shooter.stopIntake();
                return;
            }

            shootingUnlocked = true;

            stopperOpenedForShot = true;
            shooter.openStopper();
            stopperToIntakeTimer.reset();
        }

        shooter.openStopper();
    }

    private void resetShotSequence() {
        stopperOpenedForShot = false;
        shootingUnlocked = false;
        stopperToIntakeTimer.reset();
    }

    public boolean isStopperOpenedForShot() {
        return stopperOpenedForShot;
    }

    public double getStopperDelayMs() {
        return stopperToIntakeTimer.milliseconds();
    }

    @Override
    public void end(boolean interrupted) {
        resetShotSequence();
        shooter.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}