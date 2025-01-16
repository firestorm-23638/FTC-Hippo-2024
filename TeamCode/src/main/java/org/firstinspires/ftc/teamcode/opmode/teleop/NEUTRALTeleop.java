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
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@TeleOp
public class NEUTRALTeleop extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drive;
    private Intake intake;
    private Basket basket;
    private Elevator elevator;
    //private SpecimenClaw specimen;

    private Limelight limelight;

    private Pose2d basketPose;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0), telemetry);
        intake = new Intake(hardwareMap, telemetry,  Intake.color.RED, gamepad1);
        basket = new Basket(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry, gamepad1);
        //specimen = new SpecimenClaw(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);

        drive.forwardSpeedlimit = 0.675;
        drive.strafeSpeedlimit = 0.675;
        drive.rotSpeedLimit = 0.4;

        // Change the basket goal based on the alliance
        if (Constants.isRed) {
            basketPose = new Pose2d(-58.923881554, -55.0502525317, Math.toRadians(45));
        }
        else {
            basketPose = new Pose2d(58.923881554, 55.0502525317, Math.toRadians(45+180));
        }

        GamepadButton intakeOut = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton toBasket = new GamepadButton(driver, GamepadKeys.Button.A);
        GamepadButton zoomZoom = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);

        GamepadButton basketTrajectoryButton = new GamepadButton(driver, GamepadKeys.Button.Y);

        GamepadButton basketOut = new GamepadButton(operator, GamepadKeys.Button.A);
        GamepadButton depositorUp = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDown = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton depositorSlight = new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT);
        //GamepadButton elevatorTrim = new GamepadButton(driver, )

        GamepadButton elevUp = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        GamepadButton elevDown = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);

        //GamepadButton specimenClaw = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton depositorBar = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);

        GamepadButton resetGyro = new GamepadButton(driver, GamepadKeys.Button.X);

        resetGyro.whenHeld(new InstantCommand(() -> drive.setCurrentPose(new Pose2d(0, 0, Math.toRadians(270)))));

        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)-this.gamepad1.right_stick_x,
                false));

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));

        intakeOut.whenHeld(new InstantCommand(() -> {
            intake.pivotDown();
            intake.horizontalOut();
            intake.setVacuumRun();
            intake.updateColorSensor(false);

            // Slow down the drivetrain when the intake is out
            drive.forwardSpeedlimit = 0.3;
            drive.strafeSpeedlimit = 0.3;
            drive.rotSpeedLimit = 0.3;
        })).whenReleased(new InstantCommand(() -> {
            intake.pivotHome();
            intake.horizontalIn();
            intake.setVacuumStop();
            intake.updateColorSensor(false);

            drive.forwardSpeedlimit = 0.65;
            drive.strafeSpeedlimit = 0.65;
            drive.rotSpeedLimit = 0.45;
        }));

        toBasket.whenHeld(new InstantCommand(() -> {
            intake.pivotBasket();
            intake.setVacuumEject();
        })).whenReleased(new InstantCommand(() -> {
            intake.pivotHome();
            intake.setVacuumStop();
        }));

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

//        specimenClaw.whenHeld(new InstantCommand(() -> {
//            specimen.open();
//        })).whenReleased(new InstantCommand(() -> {
//            specimen.close();
//        }));

        depositorBar.whenHeld(new InstantCommand(() -> elevator.isBarState = true))
                .whenReleased(new InstantCommand(() -> {
                    elevator.isBarState = false;
                }));

        basketOut.whenHeld(new InstantCommand(() -> basket.toDeposit()))
                .whenReleased(new InstantCommand(() -> basket.toHome()));

        depositorUp.whenPressed(new InstantCommand(() -> {elevator.increaseStage(); elevator.isZeroed = false;}));
        depositorDown.whenPressed(new InstantCommand(() -> elevator.decreaseStage()));

        depositorSlight.whenHeld(new InstantCommand(() -> elevator.isSlightState = true))
                .whenReleased(new InstantCommand(() -> elevator.isSlightState = false));

        // If a subsystem has a default command, you don't need to register.
        register(intake, basket, elevator, limelight);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));

        // Set pose from when defined in autonomous
        if (Constants.pose != null) {
            drive.setCurrentPose(Constants.pose);
        }
        else {
            drive.setCurrentPose(new Pose2d(0, 0, 0));
        }

        waitForStart();
        // Put game start code here. i.e home everything
        schedule(
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            intake.horizontalIn();
                            basket.toHome();
                            elevator.currentStage = Elevator.basketState.HOME;
                            intake.pivotHome();
                        }),
                        new RunCommand(() -> {
                            elevator.setTrim(this.gamepad1.right_trigger * 15.0);
                            if ((elevator.currentStage != Elevator.basketState.HOME) && (intake.currentState == Intake.state.RESTING)) {
                                intake.currentState = Intake.state.SLIGHTLY;
                            }
                            else if ((elevator.currentStage == Elevator.basketState.HOME) && (intake.currentState == Intake.state.SLIGHTLY)) {
                                intake.currentState = Intake.state.RESTING;
                            }
                        })
                ));
    }
}
