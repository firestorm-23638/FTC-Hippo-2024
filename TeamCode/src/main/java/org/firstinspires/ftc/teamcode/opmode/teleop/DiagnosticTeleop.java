package org.firstinspires.ftc.teamcode.opmode.teleop;
//right206 2
//left105  1
//wrist 3

//0 left
//2 right
//1 bucket

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;

@TeleOp
public class DiagnosticTeleop extends CommandOpMode {
    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drive;
    private Depositor depositor;
    private Intake intake;
    private Elevator elevator;
    private Kicker kicker;
    private Limelight limelight;
    private OverflowEncoder par0;
    private OverflowEncoder par1;
    private OverflowEncoder perp;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0), telemetry);
        intake = new Intake(hardwareMap, telemetry,  Intake.color.RED, gamepad1);
        elevator = new Elevator(hardwareMap, telemetry, gamepad1);
        limelight = new Limelight(hardwareMap, telemetry);
        kicker = new Kicker(hardwareMap, telemetry);
        depositor = new Depositor(hardwareMap, telemetry);

        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontRight")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backLeft")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backRight")));

        GamepadButton frontLeft = new GamepadButton(driver, GamepadKeys.Button.X);
        GamepadButton frontRight = new GamepadButton(driver, GamepadKeys.Button.Y);
        GamepadButton backLeft = new GamepadButton(driver, GamepadKeys.Button.B);
        GamepadButton backRight = new GamepadButton(driver, GamepadKeys.Button.A);

        frontLeft.whenHeld(new InstantCommand(() -> drive.testFrontLeft(0.5)))
                .whenReleased(new InstantCommand(() -> drive.testFrontLeft(0)));

        frontRight.whenHeld(new InstantCommand(() -> drive.testFrontRight(0.5)))
                .whenReleased(new InstantCommand(() -> drive.testFrontRight(0)));

        backLeft.whenHeld(new InstantCommand(() -> drive.testBackLeft(0.5)))
                .whenReleased(new InstantCommand(() -> drive.testBackLeft(0)));

        backRight.whenHeld(new InstantCommand(() -> drive.testBackRight(0.5)))
                .whenReleased(new InstantCommand(() -> drive.testBackRight(0)));

        // experimental
        //basketTrajectoryButton.whenHeld(new StrafeToPositionCommand(basketPose, drive))
        //        .whenReleased(new RunCommand(() -> drive.driveFieldCentric(-this.gamepad1.left_stick_y, -this.gamepad1.left_stick_x, -this.gamepad1.right_stick_x)));

        // If a subsystem has a default command, you don't need to register.
        register(intake, elevator, limelight, depositor);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));
        schedule(new ParallelCommandGroup(
                new RunCommand(() -> {
                    telemetry.addData("DEAD WHEELS", "");
                    telemetry.addData("Parallel 1", par0.getPositionAndVelocity().position);
                    telemetry.addData("Parallel 2", par1.getPositionAndVelocity().position);
                    telemetry.addData("Perpendicular", par0.getPositionAndVelocity().position);
                    telemetry.addData("", "");

                    telemetry.addData("ELEVATOR", "");
                    telemetry.addData("Encoder", elevator.getPosition());
                    telemetry.addData("Max RPM", elevator.getMaxRPM());
                    telemetry.addData("Is Inverted", elevator.getInverted());
                    telemetry.addData("", "");

                    telemetry.addData("Intake", "");
                    telemetry.addData("Pivot Encoder", intake.getPivotPos());
                    telemetry.addData("Extension Encoder", intake.getExtensionPos());
                    telemetry.addData("", "");

                    telemetry.addData("Individual Tests", "");
                    telemetry.addData("X", "frontLeft");
                    telemetry.addData("Y", "frontRight");
                    telemetry.addData("B", "backLeft");
                    telemetry.addData("A", "backRight");
                })
        ));

        // Set pose from when defined in autonomous
        if (Constants.pose != null) {
            drive.setCurrentPose(Constants.pose);
        }
        else {
            drive.setCurrentPose(new Pose2d(0, 0, 0));
        }

        waitForStart();
        // Put game start code here. i.e home everything
        schedule();
    }

    public void transitionMacro() {

    }
}
