package org.firstinspires.ftc.teamcode.commands;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Configs.AutoAimConfig_Blue;
import org.firstinspires.ftc.teamcode.Configs.AutoAimConfig_Red;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem_goal;

public class AutoAimTurretCommand_goal_Blue extends CommandBase {

    private final TurretSubsystem_goal turret;
    private final PedroDriveSubsystem drive;

    public AutoAimTurretCommand_goal_Blue(TurretSubsystem_goal turret, PedroDriveSubsystem drive) {
        this.turret = turret;
        this.drive = drive;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.aimAtFieldPoint(
                drive.getPose(),
                AutoAimConfig_Blue.GOAL_X,
                AutoAimConfig_Blue.GOAL_Y
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}