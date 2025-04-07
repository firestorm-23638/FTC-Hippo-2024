package opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import commands.DrivetrainCommand;
import commands.IntakingCommand;
import subsystems.Drivetrain;
import subsystems.Intake;

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

        drive = new Drivetrain(hardwareMap, new Pose(0, 0, 0), telemetry);
        intake = new Intake(hardwareMap, telemetry,  Intake.color.RED, gamepad1);

        GamepadButton intakeOut = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);

        schedule(new RunCommand(telemetry::update));

        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)this.gamepad1.left_stick_x,
                ()->(double)this.gamepad1.right_stick_x,
                false));

        intakeOut.whenHeld(new SequentialCommandGroup(
                new IntakingCommand(intake, Intake.color.RED)
        )).whenReleased(new InstantCommand(() -> {
            intake.currentState = Intake.state.RESTING;
            intake.updateColorSensor(false);
        }));

        register(intake);
        waitForStart();

        schedule(new InstantCommand(() -> {
            drive.switchToTeleop();
        }));
    }
}
