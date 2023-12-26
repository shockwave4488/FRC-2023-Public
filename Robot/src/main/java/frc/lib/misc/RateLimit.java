package frc.lib.misc;

public class RateLimit {

  private long limit;
  private final long time;
  private long count;
  private long lastReset;

  public RateLimit(long limit, long time) {
    this.limit = limit;
    this.time = time;
  }

  public void setLimit(long limit) {
    this.limit = limit;
  }

  public long getLimit() {
    return limit;
  }

  public long getTime() {
    return time;
  }

  public boolean exec() {
    checkReset();
    if (count < limit) {
      count++;
      return true;
    }
    return false;
  }

  private void checkReset() {
    long currentTime = System.currentTimeMillis();
    if (lastReset + time < currentTime) {
      count = 0;
      lastReset = currentTime;
    }
  }
}
