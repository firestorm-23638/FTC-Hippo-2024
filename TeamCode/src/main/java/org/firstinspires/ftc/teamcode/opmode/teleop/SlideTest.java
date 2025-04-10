package org.firstinspires.ftc.teamcode.opmode.teleop;
//right206 2
//left105  1
//wrist 3

//0 left
//2 right
//1 bucket

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakingCommand;
import org.firstinspires.ftc.teamcode.commands.SlideUntilHasPieceCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class SlideTest extends CommandOpMode {
    private LynxModule lynxModule;
    private GamepadEx driver;
    private GamepadEx operator;

    private Intake intake;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        operator = new GamepadEx(this.gamepad2);

        intake = new Intake(hardwareMap, telemetry,  Intake.color.RED);

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));

        //TEST.whenHeld(new RunCommand(() -> intake.blockerDown())).whenReleased(new RunCommand(() -> intake.blockerUp()));

        // If a subsystem has a default command, you don't need to register.
        register(intake);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));

        waitForStart();

        schedule(new SequentialCommandGroup(
                new IntakePositionCommand(intake, Intake.state.INTAKING, 500, 0).withTimeout(500),
                new SlideUntilHasPieceCommand(intake, 0),
                new IntakePositionCommand(intake, Intake.state.RESTING)
            )
        );
        // Put game start code here. i.e home everything
    }
}
