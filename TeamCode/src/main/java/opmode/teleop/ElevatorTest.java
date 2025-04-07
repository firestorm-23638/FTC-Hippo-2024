package opmode.teleop;
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
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Constants.Constants;
import subsystems.Elevator;

@TeleOp
public class ElevatorTest extends CommandOpMode {
    private GamepadEx driver;

    private Elevator elev;
    private Motor motor;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);

        motor = new Motor(hardwareMap, Constants.ELEVATOR_MOTOR_CONFIG);

        GamepadButton depositorUpDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        GamepadButton depositorDownDriver = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));

        //TEST.whenHeld(new RunCommand(() -> intake.blockerDown())).whenReleased(new RunCommand(() -> intake.blockerUp()));

        depositorUpDriver.whenPressed(new InstantCommand(() -> {
            motor.set(1);
        })).whenReleased(new InstantCommand(() ->
                motor.set(0)));
        depositorDownDriver.whenPressed(new InstantCommand(() -> {
            motor.set(-1);
        })).whenReleased(new InstantCommand(() -> motor.set(0)));

        // If a subsystem has a default command, you don't need to register.
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));

        waitForStart();

        schedule(new RunCommand(() -> {
            telemetry.addData("Pos", motor.getCurrentPosition());
        }));
        // Put game start code here. i.e home everything
    }
}
