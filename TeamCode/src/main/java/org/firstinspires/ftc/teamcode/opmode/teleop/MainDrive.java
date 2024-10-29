package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.LimelightCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.security.interfaces.ECKey;

@TeleOp
public class MainDrive extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drive;
    private Intake intake;
    private Basket basket;
    private Elevator elevator;

    private Limelight limelight;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0), telemetry);
        intake = new Intake(hardwareMap, telemetry);
        basket = new Basket(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);

        limelight = new Limelight(hardwareMap, telemetry);

        GamepadButton intakeOut = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        GamepadButton toBasket = new GamepadButton(operator, GamepadKeys.Button.Y);

        GamepadButton basketOut = new GamepadButton(operator, GamepadKeys.Button.A);
        GamepadButton depositorUp = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDown = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);

        //This is okay
        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)-this.gamepad1.right_stick_x));

        // Reads limelight position for now
        limelight.setDefaultCommand(new LimelightCommand(limelight));

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

        basketOut.whenHeld(new InstantCommand(() -> basket.toDeposit()))
                .whenReleased(new InstantCommand(() -> basket.toHome()));

        depositorUp.whenPressed(new InstantCommand(() -> elevator.increaseStage()));
        depositorDown.whenPressed(new InstantCommand(() -> elevator.decreaseStage()));
        // If a subsystem has a default command, you don't need to register.
        register(intake, basket, elevator);
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
            elevator.currentStage = Elevator.state.HOME;
            intake.pivotHome();
        }));
    }
}
