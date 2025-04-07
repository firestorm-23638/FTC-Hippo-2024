package opmode.teleop;
//right206 2
//left105  1
//wrist 3

//0 left
//2 right
//1 bucket

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import commands.HorizontalTransitionCommand;
import subsystems.Depositor;
import subsystems.Elevator;
import subsystems.Intake;

@TeleOp
public class TransitionTest extends CommandOpMode {
    private GamepadEx driver;

    private Depositor dep;
    private Intake intake;
    private Elevator elevator;

    @Override
    public void initialize() {
        driver = new GamepadEx(this.gamepad1);
        dep = new Depositor(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED);
        elevator = new Elevator(hardwareMap, telemetry);

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));

        //TEST.whenHeld(new RunCommand(()
        // -> intake.blockerDown())).whenReleased(new RunCommand(() -> intake.blockerUp()));

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));

        //TEST.whenHeld(new RunCommand(() -> intake.blockerDown())).when
        // If a subsystem has a default command, you don't need to register.
        register(intake, dep);
        // Automatically updates telemetry
        schedule(new RunCommand(telemetry::update));

        waitForStart();
        schedule(new HorizontalTransitionCommand(dep, intake, elevator));
        // Put game start code here. i.e home everything
    }
}
