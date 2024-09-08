package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp
public class MainDrive extends CommandOpMode {
    private Drivetrain drive;

    @Override
    public void initialize() {
        drive = new Drivetrain(hardwareMap);
        drive.setDefaultCommand(new DrivetrainCommand(drive,
                ()->(double)-this.gamepad1.left_stick_x,
                ()->(double)-this.gamepad1.left_stick_y,
                ()->(double)-this.gamepad1.right_stick_x));

        register(drive);
    }
}
