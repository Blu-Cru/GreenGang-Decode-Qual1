package org.firstinspires.ftc.teamcode.commonA.commandsA; // Same package

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;

public class OuttakeCommandA extends CommandBase { // <-- Class name updated

    private final OuttakeA m_outtake;
    private final OuttakeA.OuttakeState m_targetState;

    public OuttakeCommandA(OuttakeA outtake, OuttakeA.OuttakeState targetState) {
        m_outtake = outtake;
        m_targetState = targetState;
        addRequirements(m_outtake);
    }

    @Override
    public void initialize() {
        m_outtake.setOuttakeState(m_targetState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}