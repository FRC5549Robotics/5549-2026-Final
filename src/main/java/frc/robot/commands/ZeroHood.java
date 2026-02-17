package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import edu.wpi.first.wpilibj.Timer;

public class ZeroHood extends Command {
    private final Timer timer = new Timer();

    private final Hood hood;

    public ZeroHood(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        System.out.println("Zeroing Hood...");
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        hood.hoodDownInitial();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() < 0.25) {
            return false;
        }

        return hood.atBottom();
    }

    @Override
    public void end(boolean interrupted) {
        hood.stop();
        hood.zeroEncoder();
        System.out.println("Hood Zeroed");
    }
}
