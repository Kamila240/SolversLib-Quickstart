package org.firstinspires.ftc.teamcode.commands;
import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class PedroDriveCommand extends CommandBase {

    private final PedroDriveSubsystem drive;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;
    //private final BooleanSupplier slowModeSupplier;
    private final boolean robotCentric;

    public static double STICK_DEADBAND = 0.06;

    public static double DRIVE_EXPO = 0.35;
    public static double TURN_EXPO = 0.65;

    public static double DRIVE_SCALE = 1.0;
    public static double STRAFE_SCALE = 1.0;
    public static double TURN_SCALE = 0.8;

    //public static double SLOW_DRIVE_SCALE = 0.35;
    //public static double SLOW_STRAFE_SCALE = 0.35;
    //public static double SLOW_TURN_SCALE = 0.15;

    public static boolean USE_TURN_SLEW = true;
    public static double MAX_TURN_STEP = 0.04;

    private double lastTurn = 0.0;

    public PedroDriveCommand(
            PedroDriveSubsystem drive,
            DoubleSupplier forwardSupplier,
            DoubleSupplier strafeSupplier,
            DoubleSupplier turnSupplier,
            //BooleanSupplier slowModeSupplier,
            boolean robotCentric
    ) {
        this.drive = drive;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.turnSupplier = turnSupplier;
        //this.slowModeSupplier = slowModeSupplier;
        this.robotCentric = robotCentric;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double forward = processDriveAxis(forwardSupplier.getAsDouble(), DRIVE_EXPO);
        double strafe = processDriveAxis(strafeSupplier.getAsDouble(), DRIVE_EXPO);
        double turn = processTurnAxis(turnSupplier.getAsDouble());


        forward *= DRIVE_SCALE;
        strafe *= STRAFE_SCALE;
        turn *= TURN_SCALE;

        if (USE_TURN_SLEW) {
            turn = slew(turn, lastTurn, MAX_TURN_STEP);
            lastTurn = turn;
        } else {
            lastTurn = turn;
        }

        drive.setTeleOpDrive(forward, strafe, turn, robotCentric);
    }

    @Override
    public void end(boolean interrupted) {
        lastTurn = 0.0;
        drive.stopDrive(robotCentric);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double processDriveAxis(double value, double expoAmount) {
        value = deadbandRescaled(value, STICK_DEADBAND);
        value = expo(value, expoAmount);
        return clamp(value, -1.0, 1.0);
    }

    private double processTurnAxis(double value) {
        value = deadbandRescaled(value, STICK_DEADBAND);
        value = expo(value, TURN_EXPO);
        return clamp(value, -1.0, 1.0);
    }

    private double deadbandRescaled(double x, double deadband) {
        if (Math.abs(x) < deadband) {
            return 0.0;
        }
        return Math.signum(x) * (Math.abs(x) - deadband) / (1.0 - deadband);
    }

    private double expo(double x, double amount) {
        return amount * x * x * x + (1.0 - amount) * x;
    }

    private double slew(double target, double current, double maxStep) {
        double delta = target - current;

        if (delta > maxStep) delta = maxStep;
        if (delta < -maxStep) delta = -maxStep;

        return current + delta;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}