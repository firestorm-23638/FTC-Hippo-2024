package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ServoTester;

@TeleOp
public class ServoTesterOpmode extends CommandOpMode {
    private ServoTester servoTester;

    private GamepadEx driver;
    private GamepadEx operator;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        servoTester = new ServoTester(hardwareMap, telemetry, new String[]{
                "basketServo",
                "intakePivot",
                "specimen",
                "leftHorizontal",
                "rightHorizontal",
        });

        GamepadButton toggleSet = new GamepadButton(driver, GamepadKeys.Button.A);
        GamepadButton degreeUp = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        GamepadButton degreeDown = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton nameLeft = new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT);
        GamepadButton nameRight = new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT);

        toggleSet.whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servoTester.toggleSet()),
                        new InstantCommand(() -> gamepad1.rumbleBlips(1))   // for effect ;)
                )
        );

        degreeUp.whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servoTester.incrementDegree()),
                        new InstantCommand(() -> gamepad1.rumbleBlips(1))
                )
        );
        degreeDown.whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servoTester.decrementDegree()),
                        new InstantCommand(() -> gamepad1.rumbleBlips(1))
                )
        );

        nameLeft.whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servoTester.decrementServo()),
                        new InstantCommand(() -> gamepad1.rumbleBlips(1))
                )
        );
        nameRight.whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servoTester.decrementDegree()),
                        new InstantCommand(() -> gamepad1.rumbleBlips(1))
                )
        );

        waitForStart();
        schedule(new RunCommand(telemetry::update));
    }
}