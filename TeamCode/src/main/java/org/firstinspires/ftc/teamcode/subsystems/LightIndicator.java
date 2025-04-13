package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class LightIndicator extends SubsystemBase {
    private Servo port0;
    private Servo port1;
    private Timing.Timer patternTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private short currentAction = 0;
    private boolean override = false;
    private PatternState overrideState = PatternState.YELLOW_BLUE;
    private Telemetry telemetry;
    private ColorState colorState;

    public PatternState patternState = PatternState.YELLOW_BLUE;


    public enum ColorState {
        RED,
        BLUE,
        YELLOW,
        WHITE,
        GREEN,
        OFF
    }

    public enum PatternState {
        SOLID_RED,
        SOLID_BLUE,
        YELLOW_RED,
        YELLOW_BLUE,
        FLASHING_GREEN,
        FLASHING_BLUE,

        OVERRIDE_GREEN,
        OVERRIDE_RED
    }

    public static class LightAction {
        public ColorState state;
        public long millis;

        public LightAction(ColorState state, long millis) {
            this.state = state;
            this.millis = millis;
        }
    }

    // fancy stuff
    public static class LightBuilder {
        private ArrayList<LightAction> action = new ArrayList<>();

        public LightBuilder color(ColorState color, long millis) {
            action.add(new LightAction(color, millis));
            return this;
        }

        public List<LightAction> build() {
            return action;
        }
    }

    public LightIndicator(HardwareMap hardwareMap, Telemetry telemetry) {
        port0 = hardwareMap.get(Servo.class, Constants.LIGHT_INDICATOR0_CONFIG);
        port1 = hardwareMap.get(Servo.class, Constants.LIGHT_INDICATOR1_CONFIG);
        this.telemetry = telemetry;
    }

    private boolean executePattern(List<LightAction> actions) {
        boolean ret = false;
        setState(actions.get(currentAction).state);
        colorState = actions.get(currentAction).state;
        if (patternTimer.done()) {
            currentAction++;
            if (currentAction >= actions.size()) {
                currentAction = 0;
                ret = true;
            }
            patternTimer = new Timing.Timer(actions.get(currentAction).millis, TimeUnit.MILLISECONDS);
            patternTimer.start();
        }
        else if (!patternTimer.isTimerOn()) {
            patternTimer = new Timing.Timer(actions.get(currentAction).millis, TimeUnit.MILLISECONDS);
            patternTimer.start();
        }
        return ret;
    }

    public void setOverridePattern(PatternState state) {
        override = true;
        overrideState = state;
    }

    @Override
    public void periodic() {
//        if (override) {
//            switch (overrideState) {
//                case OVERRIDE_GREEN:
//                    override = !executePattern(new LightBuilder()
//                            .color(ColorState.GREEN, 150)
//                            .color(ColorState.OFF, 100)
//                            .color(ColorState.GREEN, 150)
//                            .color(ColorState.OFF, 100)
//                            .build());
//                    break;
//                case OVERRIDE_RED:
//                    override = !executePattern(new LightBuilder()
//                            .color(ColorState.RED, 150)
//                            .color(ColorState.OFF, 100)
//                            .color(ColorState.RED, 150)
//                            .color(ColorState.OFF, 100)
//                            .build());
//                    break;
//            }
//        }
        telemetry.addData("pattern state", patternState);
        telemetry.addData("Current action", currentAction);
        telemetry.addData("current state", colorState);
        switch (patternState) {
            case SOLID_RED:
                setState(ColorState.RED);
                break;
            case SOLID_BLUE:
                setState(ColorState.BLUE);
                break;
            case YELLOW_RED:
                executePattern(new LightBuilder()
                        .color(ColorState.YELLOW, 500)
                        .color(ColorState.RED, 500)
                        .build());
                break;
            case YELLOW_BLUE:
                telemetry.addData("here", "");
                executePattern(new LightBuilder()
                        .color(ColorState.YELLOW, 500)
                        .color(ColorState.BLUE, 500)
                        .build());
                break;
            case FLASHING_GREEN:
                executePattern(new LightBuilder()
                        .color(ColorState.GREEN, 700)
                        .color(ColorState.OFF, 300)
                        .build());
                break;
            case FLASHING_BLUE:
                executePattern(new LightBuilder()
                        .color(ColorState.BLUE, 500)
                        .color(ColorState.OFF, 500)
                        .build());
                break;

        }
    }

    public void setPatternState(PatternState state) {
        patternState = state;
        currentAction = 0;
    }

    public void setState(ColorState s) {
        switch (s) {
            case OFF:
                turnOff();
                break;
            case RED:
                setRed();
                break;
            case WHITE:
                setWhite();
                break;
            case BLUE:
                setBlue();
                break;
            case YELLOW:
                setYellow();
                break;
            case GREEN:
                setGreen();
                break;
        }
    }

    private void turnOff() {
        port0.setPosition(0);
        port1.setPosition(0);
    }

    private void setRed() {
        port0.setPosition(0.279);
        port1.setPosition(0.279);
    }

    private void setBlue() {
        port0.setPosition(0.611);
        port1.setPosition(0.611);
    }

    private void setYellow() {
        port0.setPosition(0.388);
        port1.setPosition(0.388);
    }

    private void setWhite() {
        port0.setPosition(1);
        port1.setPosition(1);
    }

    private void setGreen() {
        port0.setPosition(0.5);
        port1.setPosition(0.5);
    }
}
