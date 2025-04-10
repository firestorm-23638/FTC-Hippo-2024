package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.IntakingCommand;
import org.firstinspires.ftc.teamcode.commands.TurnAndForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class ForwardTurnTest extends CommandOpMode {
    private Drivetrain drive;

    @Override
    public void initialize() {

        drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, Math.toRadians(45)), telemetry);

        schedule(new RunCommand(telemetry::update));

        register(drive);
        waitForStart();

        schedule(new ParallelCommandGroup(
                new TurnAndForwardCommand(drive, 0.25, 80)
//                new RunCommand(() -> {
//                    telemetry.addData("setting to", (80 - Math.toDegrees(drive.getCurrentPose().heading.log())) * -0.01);
//                })
        ));
    }
}
