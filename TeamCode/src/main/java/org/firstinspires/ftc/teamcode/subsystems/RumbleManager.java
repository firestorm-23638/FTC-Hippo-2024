package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class RumbleManager extends SubsystemBase {
    private Gamepad gamepad;

    private Timing.Timer gameTimer;
    private boolean inEndgame = false;
    private int currentSecond = 15;

    public RumbleManager(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        gameTimer = new Timing.Timer(120, TimeUnit.SECONDS);
    };

    public void startGameTimer() {
        gameTimer.start();
    }

    public void initialize() {
        startGameTimer();
    }

    @Override
    public void periodic() {
        if (gameTimer.isTimerOn()) {
            if (gameTimer.done()) {
                gamepad.rumble(1, 1, 5000);
            }
            else if (gameTimer.remainingTime() <= 15) {
                if (gameTimer.remainingTime() == currentSecond) {
                    gamepad.rumble(0.5, 0.5, 200);
                    currentSecond --;
                }
            }
            else if (gameTimer.remainingTime() <= 30) {
                if (!inEndgame) {
                    gamepad.rumble(0.5, 0.5, 500);
                    inEndgame = true;
                }
            }
        }
    }

    public void requestRumble(double left, double right, int ms) {
        gamepad.rumble(left, right, ms);
    }

    public void requestBlip(int amt) {
        gamepad.rumbleBlips(amt);
    }
}