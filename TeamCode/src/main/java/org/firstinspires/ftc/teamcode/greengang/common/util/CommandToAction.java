package org.firstinspires.ftc.teamcode.greengang.common.util;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;

public class CommandToAction implements Action {
    private final Command command;
    private boolean initialized = false;

    public CommandToAction(Command command) {
        this.command = command;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            command.initialize();
            initialized = true;
        }

        command.execute();

        if (command.isFinished()) {
            command.end(false);
            return false;
        }

        return true;
    }
}