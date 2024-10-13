package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.Supplier;

public class IntakeCommand extends CommandBase {
    private final Intake m_intake;

    private final Supplier<Boolean> intakeOutSupplier;
    private final Supplier<Boolean> testSupplier;

    public IntakeCommand(Intake intake, Supplier<Boolean> intakeOut, Supplier<Boolean> test) {
        m_intake = intake;
        intakeOutSupplier = intakeOut;
        testSupplier = test;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        //TODO make intake go in and out
    }
}
