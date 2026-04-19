package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Extension;
import edu.wpi.first.wpilibj.Timer;

public class ZeroExtensionInstant extends Command {
    private final Timer timer = new Timer();

    private final Extension extension;

    public ZeroExtensionInstant(Extension extension) {
        this.extension = extension;
        addRequirements(extension);
    }

    @Override
    public void initialize() {
        System.out.println("Zeroing extension...");
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        //extension.extensionDownSlow();
    }

    @Override
    public boolean isFinished() {
        return extension.atBottom();
    }

    @Override
    public void end(boolean interrupted) {
        //extension.stop();
        extension.zeroEncoder();
    }
}
