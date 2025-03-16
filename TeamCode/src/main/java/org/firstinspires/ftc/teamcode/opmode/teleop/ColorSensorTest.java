package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.commands.SlideUntilHasPieceCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.TwoWayColorSensor;

@TeleOp
public class ColorSensorTest extends CommandOpMode {
    private TwoWayColorSensor sensor;

    @Override
    public void initialize() {
        sensor = new TwoWayColorSensor(hardwareMap, telemetry);

        register(sensor);

        waitForStart();
        schedule(new RunCommand(telemetry::update));

        // Put game start code here. i.e home everything
        schedule(new RunCommand(() -> {
            telemetry.addData("Is Red", sensor.isRed());
            telemetry.addData("Is Blue", sensor.isBlue());
            telemetry.addData("Is Yellow", sensor.isYellow());
            telemetry.addData("Is None", sensor.isNone());
        }));
    }
}