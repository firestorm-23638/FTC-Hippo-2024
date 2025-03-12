package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.RumbleManager;

public class RumbleRawCommand extends CommandBase {
    private RumbleManager rumble;
    private double leftAmt, rightAmt;
    private int millis;

    public RumbleRawCommand(RumbleManager rumble, double leftAmt, double rightAmt, int millis) {
        this.rumble = rumble;
        this.leftAmt = leftAmt;
        this.rightAmt = rightAmt;
        this.millis = millis;
    }

    @Override
    public void execute() {
        this.rumble.requestRumble(leftAmt, rightAmt, millis);
    }

    public boolean isFinished() {
        return true;
    }
}