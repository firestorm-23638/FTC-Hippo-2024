package org.firstinspires.ftc.teamcode.opmode.teleop;
//right206 2
//left105  1
//wrist 3

//0 left
//2 right
//1 bucket

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakingCommand;
import org.firstinspires.ftc.teamcode.commands.RumbleRawCommand;
import org.firstinspires.ftc.teamcode.commands.TransitionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.RumbleManager;

@TeleOp
public class BLUETeleop extends CommandOpMode {
    private LynxModule lynxModule;
    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drive;
    private Depositor depositor;
    private Intake intake;
    private Elevator elevator;
    private Kicker kicker;
    private RumbleManager rumbleManager;

    private Limelight limelight;

    private Pose2d basketPose;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap, new Pose(0, 0, 0), telemetry);
        intake = new Intake(hardwareMap, telemetry,  Intake.color.RED, gamepad1);
        elevator = new Elevator(hardwareMap, telemetry, gamepad1);
        limelight = new Limelight(hardwareMap, telemetry);
        kicker = new Kicker(hardwareMap, telemetry);
        depositor = new Depositor(hardwareMap, telemetry);
        rumbleManager = new RumbleManager(hardwareMap, telemetry, gamepad1);

        lynxModule = hardwareMap.get(LynxModule.class, "Control Hub");

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        // Change the basket goal based on the alliance
        if (Constants.IS_RED) {
            basketPose = new Pose2d(-58.923881554, -55.0502525317, Math.toRadians(45));
        }
        else {
            basketPose = new Pose2d(58.923881554, 55.0502525317, Math.toRadians(45+180));
        }

        GamepadButton intakeOut = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton transition = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        GamepadButton zoomZoom = new GamepadButton(driver, GamepadKeys.Button.Y);
        GamepadButton driverClaw = new GamepadButton(driver, GamepadKeys.Button.A);
        GamepadButton kickerButton = new GamepadButton(driver, GamepadKeys.Button.X);
        GamepadButton ejectSample = new GamepadButton(driver, GamepadKeys.Button.B);
        GamepadButton TEST = new GamepadButton(operator, GamepadKeys.Button.Y);

        GamepadButton basketTrajectoryButton = new GamepadButton(driver, GamepadKeys.Button.Y);

        GamepadButton basketScore = new GamepadButton(operator, GamepadKeys.Button.A);
        GamepadButton specimenScore = new GamepadButton(operator, GamepadKeys.Button.X);
        GamepadButton depositorUp = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDown = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton depositorUpDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDownDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton depositorSlight = new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT);

        GamepadButton elevUp = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        GamepadButton elevDown = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);

        GamepadButton depositorBar = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);

        kickerButton.whenPressed(new InstantCommand(() -> kicker.currentState = Kicker.state.OPEN))
                .whenReleased(new InstantCommand(() -> kicker.currentState = Kicker.state.CLOSE));

        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)-this.gamepad1.right_stick_x,
                false));

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));

        //TEST.whenHeld(new RunCommand(() -> intake.blockerDown())).whenReleased(new RunCommand(() -> intake.blockerUp()));

        intakeOut.whenHeld(new SequentialCommandGroup(
                new IntakingCommand(intake, Intake.color.RED),
                new RumbleRawCommand(rumbleManager, 0.5, 0.5, 100)
        )).whenReleased(new InstantCommand(() -> {
            intake.currentState = Intake.state.RESTING;
            intake.updateColorSensor(false);

            drive.forwardSpeedlimit = 1;
            drive.strafeSpeedlimit = 1;
            drive.rotSpeedLimit = 1;
        }));

        ejectSample.whenHeld(new InstantCommand(() -> {
                intake.currentState = Intake.state.BARFING;
        })).whenReleased(new InstantCommand(() -> {
                intake.currentState = Intake.state.RESTING;
        }));

        basketScore.whenHeld(new SequentialCommandGroup(
                //new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(800),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(400)
        )).whenReleased(new SequentialCommandGroup(
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100)
                //new DepositorCommand(depositor, Depositor.state.HOME).withTimeout(800)
        ));

        driverClaw.whenHeld(new SequentialCommandGroup(
                //new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(800),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(400)
        )).whenReleased(new SequentialCommandGroup(
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100)
                //new DepositorCommand(depositor, Depositor.state.HOME).withTimeout(800)
        ));

        specimenScore.whenHeld(new SequentialCommandGroup(
                new DepositorCommand(depositor, Depositor.state.SPECIMEN).withTimeout(800),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100)
        )).whenReleased(new SequentialCommandGroup(
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(300),
                new DepositorCommand(depositor, Depositor.state.HOME).withTimeout(10)
        ));

        transition.whenHeld(
                new SequentialCommandGroup(
                    new TransitionCommand(depositor, intake, elevator),
                    new RumbleRawCommand(rumbleManager, 0.75, 0.75, 100)
                ))
                .whenReleased(new SequentialCommandGroup(
                    new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(100),
                    new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET),
                    new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(100)
                ));

        zoomZoom.whenHeld(new InstantCommand(() -> {
            drive.rotSpeedLimit = .8;
            drive.strafeSpeedlimit = 1;
            drive.forwardSpeedlimit = 1;
        })).whenReleased(new InstantCommand(() -> {
            drive.forwardSpeedlimit = 0.675;
            drive.strafeSpeedlimit = 0.675;
            drive.rotSpeedLimit = 0.4;
        }));



        // Manual test elevator
/*      elevUp.whenHeld(new InstantCommand(() -> {
            elevator.vertical.set(1);
        })).whenReleased(new InstantCommand(() -> {
            elevator.vertical.set(0);
        }));

        elevDown.whenHeld(new InstantCommand(() -> {
            elevator.vertical.set(-1);
        })).whenReleased(new InstantCommand(() -> {
            elevator.vertical.set(0);
        })); */

        depositorBar.whenHeld(new InstantCommand(() -> elevator.isBarState = true))
                .whenReleased(new InstantCommand(() -> {
                    elevator.isBarState = false;
                }));

        depositorUp.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    elevator.increaseStage();
                    elevator.isZeroed = false;
                }),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(10)
                )
        ));
        depositorDown.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    elevator.decreaseStage();
                    if (elevator.currentStage == Elevator.basketState.MIDDLE_BASKET) {
                        depositor.toHome();
                    }
                })
        ));

        depositorUpDriver.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    elevator.increaseStage();
                    elevator.isZeroed = false;
                }),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(10)
                )
        ));
        depositorDownDriver.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    elevator.decreaseStage();
                    if (elevator.currentStage == Elevator.basketState.MIDDLE_BASKET) {
                        depositor.toHome();
                    }
                })
        ));

        depositorSlight.whenHeld(new InstantCommand(() -> elevator.isSlightState = true))
                .whenReleased(new InstantCommand(() -> elevator.isSlightState = false));

        // If a subsystem has a default command, you don't need to register.
        register(intake, elevator, limelight, depositor, rumbleManager);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));

        waitForStart();
        // Put game start code here. i.e home everything
        schedule(
                new ParallelCommandGroup(
                        new RunCommand(() -> telemetry.addData("Expansion Current", lynxModule.getCurrent(CurrentUnit.MILLIAMPS))),
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(100),
                                new DepositorCommand(depositor, Depositor.state.HOME).withTimeout(100)
                        ),
                        new InstantCommand(() -> kicker.close()),
                        new InstantCommand(() -> {
                            elevator.currentStage = Elevator.basketState.MIDDLE_BASKET;
                        }),
                        new RumbleRawCommand(rumbleManager, 0.5, 0.5, 200)
                ));
    }
}
