package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.IntakingCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class DriveTest extends CommandOpMode {
    private Drivetrain drive;
    private Intake intake;

    private GamepadEx driver;
    private GamepadEx operator;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0), telemetry);
        intake = new Intake(hardwareMap, telemetry, gamepad1);
        intake.setTargetColor(Intake.color.BLUE_YELLOW);

        GamepadButton intakeOut = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);

        schedule(new RunCommand(telemetry::update));

        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)this.gamepad1.left_stick_x,
                ()->(double)this.gamepad1.right_stick_x,
                false));

        intakeOut.whenHeld(new SequentialCommandGroup(
                new IntakingCommand(intake)
        )).whenReleased(new InstantCommand(() -> {
            intake.currentState = Intake.state.RESTING;
            intake.updateColorSensor(false);
        }));

        register(intake);
        waitForStart();
    }
}
