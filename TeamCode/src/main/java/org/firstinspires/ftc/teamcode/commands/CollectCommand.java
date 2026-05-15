package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;

public class CollectCommand extends CommandBase {

    public enum Mode {
        TOTAL_FROM_START,
        EXTRA_AFTER_DRIVE_DONE
    }

    private final ShooterSubsystem shooter;
    private final BooleanSupplier driveBusy;
    private final long durationMs;
    private final Mode mode;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean arrivalDetected = false;

    public CollectCommand(
            ShooterSubsystem shooter,
            BooleanSupplier driveBusy,
            long durationMs,
            Mode mode
    ) {
        this.shooter = shooter;
        this.driveBusy = driveBusy;
        this.durationMs = durationMs;
        this.mode = mode;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        arrivalDetected = false;
        timer.reset();

        shooter.stopFlywheels();
        shooter.closeStopper();
        shooter.stopIntake();
    }

    @Override
    public void execute() {
        shooter.closeStopper();
        shooter.feedIntake();

        if (mode == Mode.EXTRA_AFTER_DRIVE_DONE) {
            if (!driveBusy.getAsBoolean() && !arrivalDetected) {
                arrivalDetected = true;
                timer.reset();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (mode == Mode.TOTAL_FROM_START) {
            return timer.milliseconds() >= durationMs;
        }

        return arrivalDetected && timer.milliseconds() >= durationMs;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopIntake();
    }
}