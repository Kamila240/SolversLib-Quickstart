package org.firstinspires.ftc.teamcode.commands;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootBurstCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final double targetVelocity;
    private final int shotCount;

    private final ElapsedTime timer = new ElapsedTime();

    private int shotsDone = 0;
    private final boolean farShot;

    private enum State {
        SPINUP,
        OPEN_DELAY,
        FEED,
        RECOVER,
        FINISHED
    }

    private State state = State.SPINUP;

    private static final double STOPPER_DELAY_MS = 80.0;
    private static final double FEED_TIME_MS = 1500.0;
    private static final double RECOVER_TIME_MS = 150.0;

    public ShootBurstCommand(
            ShooterSubsystem shooter,
            double targetVelocity,
            int shotCount,
            boolean farShot
    ) {
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;
        this.shotCount = shotCount;
        this.farShot = farShot;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shotsDone = 0;
        state = State.SPINUP;
        shooter.closeStopper();
        shooter.stopIntake();
    }

    @Override
    public void execute() {
        //shooter.runClosedLoop(targetVelocity);
        shooter.runClosedLoop(targetVelocity, farShot);

        switch (state) {
            case SPINUP:
                if (shooter.isReadyFor(targetVelocity)) {
                    shooter.openStopper();
                    timer.reset();
                    state = State.OPEN_DELAY;
                }
                break;

            case OPEN_DELAY:
                if (timer.milliseconds() >= STOPPER_DELAY_MS) {
                    shooter.feedIntake();
                    timer.reset();
                    state = State.FEED;
                }
                break;

            case FEED:
                if (timer.milliseconds() >= FEED_TIME_MS) {
                    shooter.stopIntake();
                    //shooter.closeStopper();
                    timer.reset();
                    state = State.RECOVER;
                }
                break;

            case RECOVER:
                if (timer.milliseconds() >= RECOVER_TIME_MS) {
                    shotsDone++;
                    if (shotsDone >= shotCount) {
                        state = State.FINISHED;
                    } else {
                        state = State.SPINUP;
                    }
                }
                break;

            case FINISHED:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopIntake();
        shooter.closeStopper();
        shooter.stopFlywheels();
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }
}