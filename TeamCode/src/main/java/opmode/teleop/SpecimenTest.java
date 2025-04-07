package opmode.teleop;
//right206 2
//left105  1
//wrist 3

//0 left
//2 right
//1 bucket

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import commands.DepositorCommand;
import commands.DrivetrainCommand;
import commands.ElevatorPositionCommand;
import commands.HorizontalTransitionCommand;
import commands.IntakePositionCommand;
import commands.IntakingCommand;
import commands.PathChainCommand;
import commands.RumbleRawCommand;
import commands.VerticalTransitionCommand;
import subsystems.Depositor;
import subsystems.Drivetrain;
import subsystems.Elevator;
import subsystems.Intake;
import subsystems.RumbleManager;

@TeleOp
public class SpecimenTest extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drive;
    private Depositor dep;
    private Intake intake;
    private Elevator elevator;
    private RumbleManager rumbleManager;

    private boolean isVerticalTransition = false;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap, new Pose(0, 0, 0), telemetry);
        dep = new Depositor(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED, gamepad1);
        elevator = new Elevator(hardwareMap, telemetry);
        rumbleManager = new RumbleManager(hardwareMap, telemetry, gamepad1);

        GamepadButton transition = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton intakeButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton depositorUpDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDownDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);

        GamepadButton score = new GamepadButton(operator, GamepadKeys.Button.A);
        GamepadButton switchTransition = new GamepadButton(operator, GamepadKeys.Button.Y);

        transition.whenPressed(
                new ConditionalCommand(
                        new HorizontalTransitionCommand(dep, intake, elevator),
                        new VerticalTransitionCommand(dep, intake, elevator),
                        () -> isVerticalTransition
                )).whenReleased(
                        new ParallelCommandGroup(
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                new DepositorCommand(dep, Depositor.state.BUCKET)
                        )
        );

        intakeButton.whenHeld(
                new SequentialCommandGroup(
                        new DepositorCommand(dep, Depositor.state.CLAWOPEN).withTimeout(10),
                        new DepositorCommand(dep, Depositor.state.SPECIMEN).withTimeout(100)
                )
        ).whenReleased(
                new SequentialCommandGroup(
                        new DepositorCommand(dep, Depositor.state.CLAWTIGHTEN).withTimeout(100),
                        new DepositorCommand(dep, Depositor.state.PRIME_SPECIMEN).withTimeout(400),
                        new DepositorCommand(dep, Depositor.state.PLACE_SPECIMEN).withTimeout(500)
                )
        );

        depositorUpDriver.whenPressed(new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN))
                .whenReleased(
                        new SequentialCommandGroup(
                                new DepositorCommand(dep, Depositor.state.SCORE_SPECIMEN).withTimeout(100),
                                new DepositorCommand(dep, Depositor.state.CLAWOPEN).withTimeout(100),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        ));

        // OPERATOR

        switchTransition.whenPressed(new InstantCommand(() -> isVerticalTransition = !isVerticalTransition));  // Switches transition mode

        score.whenHeld(new DepositorCommand(dep, Depositor.state.CLAWOPEN));

        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)this.gamepad1.right_stick_x,
                false));

        register(intake, dep, elevator, rumbleManager);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));

        waitForStart();

        schedule(
                new SequentialCommandGroup(
                        new IntakePositionCommand(intake, Intake.state.SPECIMEN).withTimeout(10),
                        new InstantCommand(() -> drive.setToBrakeMode()),
                        new DepositorCommand(dep, Depositor.state.CLAWOPEN).withTimeout(10),
                        new InstantCommand(() -> dep.toPrime1()),
                        new InstantCommand(() -> drive.switchToTeleop()),
                        new RumbleRawCommand(rumbleManager, 0.5, 0.5, 200).withTimeout(10).withTimeout(100)
                )
        );
        // Put game start code here. i.e home everything
    }
}