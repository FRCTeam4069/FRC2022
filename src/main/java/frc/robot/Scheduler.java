package frc.robot;

import java.util.ArrayList;

public class Scheduler {

    private ArrayList<RobotRepeatingTask> repeatingTasks = new ArrayList<>();

    // Runs tasks
    void runRepeatingTasks() {
        for (RobotRepeatingTask task : repeatingTasks)
            task.run();    
    }

    // Starts an async repeating task
    void launchAsyncTask(RobotAsyncTask task) {
        new Thread() {
            @Override
            public void run() {
                task.run();
            }
        }.start();
    }

    /** Schedules a task */
    public void schedule(RobotRunnableIface task) {
        if (task instanceof RobotAsyncTask)
            launchAsyncTask((RobotAsyncTask) task);
        else if (task instanceof RobotRepeatingTask)
            repeatingTasks.add((RobotRepeatingTask) task);
    }

    /** Runnable */
    @FunctionalInterface
    public static interface RobotRunnableIface extends Runnable {}

    /** Task to run asynchronously */
    public static interface RobotAsyncTask extends RobotRunnableIface {}

    /** Repeating task to run every robot tick in main thread */
    public static interface RobotRepeatingTask extends RobotRunnableIface {}
    
}
