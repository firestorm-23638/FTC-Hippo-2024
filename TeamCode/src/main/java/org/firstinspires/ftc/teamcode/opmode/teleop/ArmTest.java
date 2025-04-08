package org.firstinspires.ftc.teamcode.opmode.teleop;
//right206 2
//left105  1
//wrist 3

//0 left
//2 right
//1 bucket

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;

@TeleOp
public class ArmTest extends CommandOpMode {
    private GamepadEx driver;

    private Depositor dep;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);

        dep = new Depositor(hardwareMap, telemetry);

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));

        //TEST.whenHeld(new RunCommand(()
        // -> intake.blockerDown())).whenReleased(new RunCommand(() -> intake.blockerUp()));
        GamepadButton shoulderInc = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        GamepadButton shoulderDec = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        GamepadButton elbowInc = new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT);
        GamepadButton elbowDec = new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT);

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));

        //TEST.whenHeld(new RunCommand(() -> intake.blockerDown())).whenReleased(new RunCommand(() -> intake.blockerUp()));

        shoulderInc.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    dep.currentShoulder ++;
                })
        ));
        shoulderDec.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    dep.currentShoulder --;
                })
        ));
        elbowInc.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    dep.currentElbow ++;
                })
        ));
        elbowDec.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    dep.currentElbow --;
                })
        ));
        // If a subsystem has a default command, you don't need to register.
        register(dep);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));

        waitForStart();
        schedule(new RunCommand(() -> {
            telemetry.addData("shoulder", dep.currentShoulder);
            telemetry.addData("elbow", dep.currentElbow);
        }));
        // Put game start code here. i.e home everything
    }
}
