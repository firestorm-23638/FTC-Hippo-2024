package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class MainDrive extends CommandOpMode {
    private Drivetrain drive;
    private Intake intake;
    private Depositor depositor;

    @Override
    public void initialize() {

        drive = new Drivetrain(hardwareMap);
        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.right_stick_x));

        intake = new Intake(hardwareMap, telemetry);
        intake.setDefaultCommand(new IntakeCommand(intake,
                ()->this.gamepad1.right_bumper,
                ()->this.gamepad1.left_bumper));

        depositor = new Depositor(hardwareMap, telemetry);
        depositor.setDefaultCommand(new DepositorCommand(depositor,
                ()->this.gamepad1.dpad_up,
                ()->this.gamepad1.dpad_down));

        register(drive, intake, depositor);

        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));
    }
}
