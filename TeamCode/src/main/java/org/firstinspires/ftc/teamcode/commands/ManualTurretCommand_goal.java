package org.firstinspires.ftc.teamcode.commands;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem_goal;
import java.util.function.DoubleSupplier;

public class ManualTurretCommand_goal extends CommandBase {

    private final TurretSubsystem_goal turret;
    private final DoubleSupplier stickSupplier;

    public ManualTurretCommand_goal(TurretSubsystem_goal turret, DoubleSupplier stickSupplier) {
        this.turret = turret;
        this.stickSupplier = stickSupplier;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.manualControl(stickSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}