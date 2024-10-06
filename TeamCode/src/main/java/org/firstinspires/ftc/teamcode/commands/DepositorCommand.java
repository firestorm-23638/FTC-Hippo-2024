package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.Supplier;

public class DepositorCommand extends CommandBase {
    private final Depositor m_depositor;

    private final Supplier<Boolean> depositorUpSupplier;
    private final Supplier<Boolean> depositorDownSupplier;

    public DepositorCommand(Depositor depositor, Supplier<Boolean> depositorUp, Supplier<Boolean> depositorDown) {
        m_depositor = depositor;
        depositorUpSupplier = depositorUp;
        depositorDownSupplier = depositorDown;

        addRequirements(m_depositor);
    }

    @Override
    public void execute() {
        m_depositor.printBasketPose();
        m_depositor.printVerticalPose();
    }
}
