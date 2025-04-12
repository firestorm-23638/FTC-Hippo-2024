package org.firstinspires.ftc.teamcode.opmode.teleop;
//right206 2
//left105  1
//wrist 3

//0 left
//2 right
//1 bucket

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.HorizontalTransitionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakingCommand;
import org.firstinspires.ftc.teamcode.commands.KickerCommand;
import org.firstinspires.ftc.teamcode.commands.LightIndicatorCommand;
import org.firstinspires.ftc.teamcode.commands.RumbleRawCommand;
import org.firstinspires.ftc.teamcode.commands.SpeedyTransitionCommand;
import org.firstinspires.ftc.teamcode.commands.VerticalTransitionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.LightIndicator;
import org.firstinspires.ftc.teamcode.subsystems.RumbleManager;

@TeleOp
public class ControlsTest extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drive;
    private Depositor dep;
    private Intake intake;
    private Elevator elevator;
    private RumbleManager rumbleManager;
    private Kicker kicker;
    private LightIndicator indicator;

    private boolean isVerticalTransition = false;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0), telemetry);
        dep = new Depositor(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED, gamepad1);
        elevator = new Elevator(hardwareMap, telemetry);
        rumbleManager = new RumbleManager(hardwareMap, telemetry, gamepad1);
        kicker = new Kicker(hardwareMap, telemetry);
        indicator = new LightIndicator(hardwareMap, telemetry);


        GamepadButton transition = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton intakeButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton depositorUp = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDown = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton depositorUpDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDownDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton kickerOut = new GamepadButton(driver, GamepadKeys.Button.X);

        GamepadButton score = new GamepadButton(driver, GamepadKeys.Button.A);
        GamepadButton switchTransition = new GamepadButton(operator, GamepadKeys.Button.Y);

        transition.whenPressed(
                new ConditionalCommand(
                        new HorizontalTransitionCommand(dep, intake, elevator),
                        new SequentialCommandGroup(
                                new SpeedyTransitionCommand(dep, intake, elevator),
                                new DepositorCommand(dep, Depositor.state.PRIME_BASKET)
                        ),
                        () -> isVerticalTransition
                )).whenReleased(
                    new SequentialCommandGroup(
                            new DepositorCommand(dep, Depositor.state.BUCKET).withTimeout(400),
                            new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                    )
        );

        intakeButton.whenHeld(
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            // Slow down the drivetrain when the intake is out
                            drive.forwardSpeedlimit = 0.3;
                            drive.strafeSpeedlimit = 0.3;
                            drive.rotSpeedLimit = 0.3;
                        }),
                        new IntakingCommand(intake, Intake.color.RED)
                )
        ).whenReleased(new InstantCommand(() -> {
            intake.currentState = Intake.state.RESTING;
            intake.updateColorSensor(false);

            drive.forwardSpeedlimit = 1;
            drive.strafeSpeedlimit = 1;
            drive.rotSpeedLimit = 1;
        }));

        depositorUp.whenPressed(
                new SequentialCommandGroup(
                        new DepositorCommand(dep, Depositor.state.BUCKET).withTimeout(400),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                )
        );
        depositorDown.whenPressed(
                new SequentialCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                        new DepositorCommand(dep, Depositor.state.PRIME1)
                )
        );

        depositorUpDriver.whenPressed(
                new SequentialCommandGroup(
                        new DepositorCommand(dep, Depositor.state.BUCKET).withTimeout(400),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                )
        );
        depositorDownDriver.whenPressed(
                new SequentialCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                        new DepositorCommand(dep, Depositor.state.PRIME1)
                )
        );

        kickerOut.whenPressed(new KickerCommand(kicker, Kicker.state.OPEN))
                .whenReleased(new KickerCommand(kicker, Kicker.state.CLOSE));


        // OPERATOR

        switchTransition.whenPressed(new InstantCommand(() -> isVerticalTransition = !isVerticalTransition));  // Switches transition mode

        score.whenHeld(new DepositorCommand(dep, Depositor.state.CLAWOPEN))
                .whenReleased(new SequentialCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                        new DepositorCommand(dep, Depositor.state.PRIME1)
                ));

        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)-this.gamepad1.right_stick_x,
                false));

        register(intake, dep, elevator, rumbleManager);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));

        waitForStart();

        schedule(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(dep, Depositor.state.CLAWOPEN).withTimeout(10),
                                new DepositorCommand(dep, Depositor.state.PRIME1).withTimeout(10),
                                new RumbleRawCommand(rumbleManager, 0.5, 0.5, 200).withTimeout(10).withTimeout(100)
                        ),
                        new LightIndicatorCommand(indicator, LightIndicator.state.YELLOW)
                )

        );
        // Put game start code here. i.e home everything
    }
}