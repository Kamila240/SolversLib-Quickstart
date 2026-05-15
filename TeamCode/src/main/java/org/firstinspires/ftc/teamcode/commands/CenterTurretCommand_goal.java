package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem_goal;

public class CenterTurretCommand_goal extends InstantCommand {

    public CenterTurretCommand_goal(TurretSubsystem_goal turret) {
        super(turret::center, turret);
    }
}