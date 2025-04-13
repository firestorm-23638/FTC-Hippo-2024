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

import org.firstinspires.ftc.teamcode.commands.ClimbCommand;
import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.HorizontalTransitionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakingCommand;
import org.firstinspires.ftc.teamcode.commands.KickerCommand;
import org.firstinspires.ftc.teamcode.commands.RumbleRawCommand;
import org.firstinspires.ftc.teamcode.commands.SpeedyTransitionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.LightIndicator;
import org.firstinspires.ftc.teamcode.subsystems.RumbleManager;

@TeleOp
public class BlueTeleop extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drive;
    private Depositor dep;
    private Intake intake;
    private Elevator elevator;
    private RumbleManager rumbleManager;
    private Kicker kicker;
    private LightIndicator indicator;
    private Climb climb;

    private TeleopState state = TeleopState.SAMPLE_MODE;

    private boolean isVerticalTransition = false;

    public enum TeleopState {
        SAMPLE_MODE,
        SPECIMEN_INTAKE,
        SPECIMEN_SCORE,
        CLIMB
    }

    public void cycleNextTeleopState() {
        switch (state) {
            case SAMPLE_MODE:
                state = TeleopState.SPECIMEN_INTAKE;
                indicator.setPatternState(LightIndicator.PatternState.SOLID_BLUE);
                intake.setTargetColor(Intake.color.BLUE);
                break;
            case SPECIMEN_INTAKE:
                state = TeleopState.SPECIMEN_SCORE;
                indicator.setPatternState(LightIndicator.PatternState.FLASHING_BLUE);
                break;
            case SPECIMEN_SCORE:
                state = TeleopState.CLIMB;
                indicator.setPatternState(LightIndicator.PatternState.FLASHING_GREEN);
                dep.toPosition(Depositor.state.BUCKET);
                break;
            case CLIMB:
                state = TeleopState.SAMPLE_MODE;
                indicator.setPatternState(LightIndicator.PatternState.YELLOW_BLUE);
                intake.setTargetColor(Intake.color.BLUE_YELLOW);
                break;
        }

    }

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0), telemetry);
        dep = new Depositor(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, gamepad1);
        elevator = new Elevator(hardwareMap, telemetry, gamepad2);
        rumbleManager = new RumbleManager(hardwareMap, telemetry, gamepad1);
        kicker = new Kicker(hardwareMap, telemetry);
        indicator = new LightIndicator(hardwareMap, telemetry);
        climb = new Climb(hardwareMap, telemetry);

        GamepadButton transition = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton intakeButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton depositorUp = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDown = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton depositorUpDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDownDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton kickerOut = new GamepadButton(driver, GamepadKeys.Button.X);
        GamepadButton snapTo45 = new GamepadButton(driver, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        GamepadButton holdClimb = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton score = new GamepadButton(operator, GamepadKeys.Button.A);
        GamepadButton switchTeleopMode = new GamepadButton(operator, GamepadKeys.Button.Y);

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
                            new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(10),
                            new DepositorCommand(dep, Depositor.state.BUCKET).withTimeout(400),
                            new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                    )
        );

        intakeButton.whenHeld(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    // Slow down the drivetrain when the intake is out
                                    drive.forwardSpeedlimit = 0.3;
                                    drive.strafeSpeedlimit = 0.3;
                                    drive.rotSpeedLimit = 0.3;
                                }),
                                new IntakingCommand(intake),
                                new RumbleRawCommand(rumbleManager, 0.8, 0.8, 400),
                                new InstantCommand(() -> {
                                    indicator.setOverridePattern(LightIndicator.PatternState.FLASHING_GREEN);
                                })
                        ),
                        new SequentialCommandGroup(
                                new DepositorCommand(dep, Depositor.state.CLAWOPEN).withTimeout(10),
                                new DepositorCommand(dep, Depositor.state.INTAKE_SPECIMEN).withTimeout(100)
                        ),
                        () -> (state == TeleopState.SAMPLE_MODE) || (state == TeleopState.SPECIMEN_INTAKE)
                )
        ).whenReleased(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            intake.currentState = Intake.state.RESTING;
                            intake.updateColorSensor(false);

                            drive.forwardSpeedlimit = 1;
                            drive.strafeSpeedlimit = 1;
                            drive.rotSpeedLimit = 1;
                        })
                ),
                new SequentialCommandGroup(
                        new DepositorCommand(dep, Depositor.state.CLAWTIGHTEN).withTimeout(100),
                        new DepositorCommand(dep, Depositor.state.PRIME_SPECIMEN).withTimeout(400),
                        new DepositorCommand(dep, Depositor.state.PLACE_SPECIMEN).withTimeout(500)
                ),
                () -> (state == TeleopState.SAMPLE_MODE) || (state == TeleopState.SPECIMEN_INTAKE)
        ));
/*new InstantCommand(() -> {
            intake.currentState = Intake.state.RESTING;
            intake.updateColorSensor(false);

            drive.forwardSpeedlimit = 1;
            drive.strafeSpeedlimit = 1;
            drive.rotSpeedLimit = 1;
        })*/
        depositorUp.whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new DepositorCommand(dep, Depositor.state.BUCKET).withTimeout(400),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                        ),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        () -> state != TeleopState.SPECIMEN_SCORE
                )
        )
                .whenReleased(
                        new ConditionalCommand(
                                new InstantCommand(),
                                new SequentialCommandGroup(
                                        new DepositorCommand(dep, Depositor.state.SCORE_SPECIMEN).withTimeout(100),
                                        new DepositorCommand(dep, Depositor.state.CLAWOPEN).withTimeout(100),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                ),
                                () -> state != TeleopState.SPECIMEN_SCORE
                        )
                );
        depositorDown.whenPressed(
                new SequentialCommandGroup(
                        new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(10),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                        new DepositorCommand(dep, Depositor.state.PRIME1)
                )
        );

        snapTo45.whenPressed(
                new InstantCommand(() -> {
                    drive.enableDriveTargetAngle(45);
                })
        ).whenReleased(new InstantCommand(() -> {
                    drive.disableDriveTargetAngle();
                }));

        depositorUpDriver.whenPressed(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DepositorCommand(dep, Depositor.state.BUCKET).withTimeout(400),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                ),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                                () -> state != TeleopState.SPECIMEN_SCORE
                        )
                )
                .whenReleased(
                        new ConditionalCommand(
                                new InstantCommand(),
                                new SequentialCommandGroup(
                                        new DepositorCommand(dep, Depositor.state.SCORE_SPECIMEN).withTimeout(100),
                                        new DepositorCommand(dep, Depositor.state.CLAWOPEN).withTimeout(100),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                ),
                                () -> state != TeleopState.SPECIMEN_SCORE
                        )
                );
        depositorDownDriver.whenPressed(
                new SequentialCommandGroup(
                        new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(10),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                        new DepositorCommand(dep, Depositor.state.PRIME1)
                )
        );

        kickerOut.whenPressed(new KickerCommand(kicker, Kicker.state.OPEN))
                .whenReleased(new KickerCommand(kicker, Kicker.state.CLOSE));


        // OPERATOR

        switchTeleopMode.whenPressed(new InstantCommand(this::cycleNextTeleopState));  // Switches transition mode

        score.whenHeld(new DepositorCommand(dep, Depositor.state.CLAWOPEN))
                .whenReleased(new SequentialCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                        new DepositorCommand(dep, Depositor.state.PRIME1)
                ));

        holdClimb.whenPressed(new InstantCommand(() -> {
            climb.toggleHoldLeftMotor();
            climb.toggleHoldRightMotor();
        }));

        // DEFAULTS

        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)-this.gamepad1.right_stick_x,
                false));

        climb.setDefaultCommand(new ClimbCommand(
                climb,
                ()->(double)(this.gamepad2.left_stick_y - this.gamepad2.left_stick_x*0.25),
                ()->(double)(this.gamepad2.left_stick_y + this.gamepad2.left_stick_x*0.25)
                ));

        register(intake, dep, elevator, rumbleManager, indicator);
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
                        new InstantCommand(() -> {
                            indicator.setPatternState(LightIndicator.PatternState.YELLOW_BLUE);
                            intake.setTargetColor(Intake.color.BLUE_YELLOW);
                            rumbleManager.start();
                        }),
                        new RunCommand(() -> {
                            telemetry.addData("Teleop state", state);
                        })
                )
        );
        // Put game start code here. i.e home everything
    }
}