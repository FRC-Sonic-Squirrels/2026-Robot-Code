package frc.lib.team2930;

import edu.wpi.first.wpilibj.RobotController;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class AutoLock implements AutoCloseable {

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal averageTime;
    public LoggerEntry.Integer timeViolation;
    public LoggerEntry.Integer count;
    public LoggerEntry.Integer countViolations;
  }

  public final Logs logs;

  private final Lock state = new ReentrantLock();
  private final int  waitThreshold;
  private int        counter;
  private int        counterViolations;
  private long       totalTime;

  public AutoLock(String name, int waitThreshold) {
    this.waitThreshold = waitThreshold;
    this.logs          = LoggerGroup.build("AutoLock").newAutoLogger(Logs.class, name);
  }

  public AutoLock lock() {
    var startTime = RobotController.getFPGATime();
    state.lock();
    counter++;
    var endTime = RobotController.getFPGATime();
    var time    = endTime - startTime;
    totalTime += time;
    if (time > waitThreshold) {
      counterViolations++;

      logs.averageTime.info((double) totalTime / counter);
      logs.count.info(counter);

      logs.timeViolation.info(time);
      logs.countViolations.info(counterViolations);
    }

    if (counter % 1024 == 0) {
      logs.count.info(counter);
      logs.averageTime.info((double) totalTime / counter);
    }

    return this;
  }

  public void close() {
    state.unlock();
  }
}
