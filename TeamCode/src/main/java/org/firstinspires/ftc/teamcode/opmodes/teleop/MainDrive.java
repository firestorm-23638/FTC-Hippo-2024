package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class MainDrive extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drive;
    private Intake intake;
    private Depositor depositor;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        depositor = new Depositor(hardwareMap, telemetry);

        GamepadButton intakeButton = new GamepadButton(operator, GamepadKeys.Button.A);
        GamepadButton depositorUp = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDown = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);

        //This is okay
        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.right_stick_x));



        // Avoid having functions in the subsystem that process user input.
        // We want to keep them reusable for auto.
        intakeButton.whenHeld(new InstantCommand(() -> {
            intake.horizontalOut();
            intake.pivotDown();
        })).whenInactive(new InstantCommand(() -> {
            intake.pivotUp();
            intake.horizontalIn();
        }));

        // We can also setup buttons to do actions when they are released.
        depositorUp.whenHeld(new InstantCommand(() -> depositor.moveVertical(0.5)))
                .whenReleased(new InstantCommand(() -> depositor.moveVertical(0)));

        depositorDown.whenHeld(new InstantCommand(() -> depositor.moveVertical(-0.5)))
                .whenReleased(new InstantCommand(() -> depositor.moveVertical(0)));

        // If a subsystem has a default command, you don't need to register.
        register(intake, depositor);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));


        waitForStart();
        // Put game start code here. i.e home everything 
        schedule(new InstantCommand(() -> {
            intake.pivotUp();
            intake.horizontalIn();
            depositor.basketToHome();
        }));
    }
}
