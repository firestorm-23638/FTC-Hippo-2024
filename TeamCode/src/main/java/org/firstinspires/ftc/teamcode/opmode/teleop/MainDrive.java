package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw;

@TeleOp
public class MainDrive extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drive;
    private Intake intake;
    private Basket basket;
    private Elevator elevator;
    private SpecimenClaw specimen;

    //private Limelight limelight;

    private Pose2d basketPose;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0), telemetry);
        intake = new Intake(hardwareMap, telemetry);
        basket = new Basket(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        specimen = new SpecimenClaw(hardwareMap, telemetry);

        //limelight = new Limelight(hardwareMap, telemetry);

        GamepadButton intakeOut = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton toBasket = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);

        GamepadButton basketOut = new GamepadButton(operator, GamepadKeys.Button.A);
        GamepadButton depositorUp = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDown = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton depositorSlight = new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT);

        GamepadButton specimenClaw = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton depositorBar = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);

        GamepadButton resetGyro = new GamepadButton(driver, GamepadKeys.Button.X);

        // Change the basket goal based on the alliance
        if (Constants.isRed) {
            basketPose = new Pose2d(-58.923881554, -55.0502525317, Math.toRadians(45));
        }
        else {
            basketPose = new Pose2d(58.923881554, 55.0502525317, Math.toRadians(45+180));
        }
        // experimental
        /*basketTrajectoryButton.whenHeld(new StrafeToPositionCommand(basketPose, drive))
                .whenReleased(new DrivetrainCommand(drive,
                        ()->(double)-this.gamepad1.left_stick_y,
                        ()->(double)-this.gamepad1.left_stick_x,
                        ()->(double)-this.gamepad1.right_stick_x));*/

        resetGyro.whenHeld(new InstantCommand(() -> drive.setCurrentPose(new Pose2d(0, 0, Math.toRadians(270)))));

        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)-this.gamepad1.right_stick_x));

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));

        intakeOut.whenHeld(new InstantCommand(() -> {
            intake.pivotDown();
            intake.horizontalOut();
            intake.vacuumRun();
        })).whenReleased(new InstantCommand(() -> {
            intake.pivotHome();
            intake.horizontalIn();
            intake.vacuumStop();
        }));

        toBasket.whenHeld(new InstantCommand(() -> {
            intake.pivotBasket();
            intake.vacuumEject();
        })).whenReleased(new InstantCommand(() -> {
            intake.pivotHome();
            intake.vacuumStop();
        }));

        specimenClaw.whenHeld(new InstantCommand(() -> {
            specimen.open();
        })).whenReleased(new InstantCommand(() -> {
            specimen.close();
        }));

        depositorBar.whenHeld(new InstantCommand(() -> elevator.isBarState = true))
                .whenReleased(new InstantCommand(() -> {
                    elevator.isBarState = false;
                    specimen.open();
                }));

        basketOut.whenHeld(new InstantCommand(() -> basket.toDeposit()))
                .whenReleased(new InstantCommand(() -> basket.toHome()));

        depositorUp.whenPressed(new InstantCommand(() -> elevator.increaseStage()));
        depositorDown.whenPressed(new InstantCommand(() -> elevator.decreaseStage()));
        depositorSlight.whenHeld(new InstantCommand(() -> elevator.isSlightState = true))
                .whenReleased(new InstantCommand(() -> elevator.isSlightState = false));

        // If a subsystem has a default command, you don't need to register.
        register(intake, basket, elevator, specimen);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));

        if (Constants.pose != null) {
            drive.setCurrentPose(Constants.pose);
        }
        else {
            drive.setCurrentPose(new Pose2d(0, 0, 0));
        }

        waitForStart();
        // Put game start code here. i.e home everything
        schedule(new InstantCommand(() -> {
            intake.horizontalIn();
            basket.toHome();
            elevator.currentStage = Elevator.basketState.HOME;
            intake.pivotHome();
        }));
    }
}
