package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Robot;

public class AutoScheduler {
    List<Command> commandQueue = new ArrayList<>();
    private Robot robot;


    boolean started = false;

    //Take in subsystems and pass them to all commands
    public AutoScheduler(Robot robot) {
        this.robot = robot;
    }

    //Add command to queue
    public void addCommand(Command command) {
        commandQueue.add(command); //Queue of type ArrayList
        command.setSubsystems(robot);
    }

    public void setCommand(int index, Command command) {
        commandQueue.add(index, command);
    }

    public int getQueueSize() {
        return commandQueue.size();
    }

    //Method takes first command in queue, when it is finished, remove it and access next in line
    public void run() {
        if(commandQueue.size() != 0) {
            Command nextCommand = commandQueue.get(0);

            if(!started) {
                nextCommand.start();
                started = true;
            }

            nextCommand.loop();
            if(nextCommand.isFinished()) {
                commandQueue.remove(0);
                if(commandQueue.size() != 0) {
                    commandQueue.get(0).start();
                }
            }
        }
    }
}
