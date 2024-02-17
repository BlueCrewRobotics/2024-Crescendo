//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.autos;

import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.List;
import java.util.function.LongConsumer;

public class SequentialCommandGroup extends Command {
    final List<Command> m_commands = new ArrayList<>();
    int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;
    private Command.InterruptionBehavior m_interruptBehavior;

    public SequentialCommandGroup(Command... commands) {
        this.m_interruptBehavior = InterruptionBehavior.kCancelIncoming;
        this.addCommands(commands);
    }

    public final void addCommands(Command... commands) {
        if (this.m_currentCommandIndex != -1) {
            throw new IllegalStateException("Commands cannot be added to a composition while it's running");
        } else {
            CommandScheduler.getInstance().registerComposedCommands(commands);
            Command[] var2 = commands;
            int var3 = commands.length;

            for(int var4 = 0; var4 < var3; ++var4) {
                Command command = var2[var4];
                this.m_commands.add(command);
                this.m_requirements.addAll(command.getRequirements());
                this.m_runWhenDisabled &= command.runsWhenDisabled();
                if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                    this.m_interruptBehavior = InterruptionBehavior.kCancelSelf;
                }
            }

        }
    }

    public void initialize() {
        this.m_currentCommandIndex = 0;
        if (!this.m_commands.isEmpty()) {
            ((Command)this.m_commands.get(0)).initialize();
        }
    }

    public final void execute() {
        if (!this.m_commands.isEmpty()) {
            Command currentCommand = (Command)this.m_commands.get(this.m_currentCommandIndex);
            currentCommand.execute();
            if (currentCommand.isFinished()) {
                currentCommand.end(false);
                ++this.m_currentCommandIndex;
                if (this.m_currentCommandIndex < this.m_commands.size()) {
                    ((Command)this.m_commands.get(this.m_currentCommandIndex)).initialize();
                }
            }

        }
    }

    public final void end(boolean interrupted) {
        if (interrupted && !this.m_commands.isEmpty() && this.m_currentCommandIndex > -1 && this.m_currentCommandIndex < this.m_commands.size()) {
            ((Command)this.m_commands.get(this.m_currentCommandIndex)).end(true);
        }

        this.m_currentCommandIndex = -1;
    }

    public final boolean isFinished() {
        return this.m_currentCommandIndex == this.m_commands.size();
    }

    public boolean runsWhenDisabled() {
        return this.m_runWhenDisabled;
    }

    public Command.InterruptionBehavior getInterruptionBehavior() {
        return this.m_interruptBehavior;
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addIntegerProperty("index", () -> {
            return (long)this.m_currentCommandIndex;
        }, (LongConsumer)null);
    }
}
