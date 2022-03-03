package frc.robot;

import java.util.ArrayList;

public class Scheduler {

    private ArrayList<RobotRepeatingTask> repeatingTasks = new ArrayList<>();

    // Runs tasks
    void runRepeatingTasks() {
        ArrayList<RobotRepeatingTask> removal = new ArrayList<>();
        // Run tasks
        for (RobotRepeatingTask task : repeatingTasks) {
            task.run();
            if (task.disabled)
                removal.add(task);
        }
        // Purge disabled
        for (RobotRepeatingTask task : removal)
            repeatingTasks.remove(task);
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
    private static interface RobotRunnableIface extends Runnable {}

    /** Task to run asynchronously */
    public interface RobotAsyncTask extends RobotRunnableIface {}

    /** Repeating task to run every robot tick in main thread*/
    public abstract static class RobotRepeatingTask implements RobotRunnableIface {
        private boolean disabled = false;

        /** Stop running the repeating task */
        public void disable() {
            disabled = true;
        }

        /** Is repeating task disabled */
        public boolean isDisabled() {
            return disabled;
        }
    }
    
}
