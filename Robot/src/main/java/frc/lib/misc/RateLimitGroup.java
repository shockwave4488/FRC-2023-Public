package frc.lib.misc;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class RateLimitGroup<T> {

  @FunctionalInterface
  public interface Distributor<T> {
    /**
     * Evenly distributes the internal limits
     *
     * @return The distributor
     */
    public static Distributor<?> makeEven() {
      return (global, internal) -> {
        long limit = global.getLimit() / (internal.size() + 1);
        internal.forEach((key, internalLimit) -> internalLimit.setLimit(limit));
        return limit;
      };
    }

    /**
     * Sets the limits to a percentage of the global limit This doesn't guarantee all internal
     * limits will be able to run
     *
     * @param percent The percent (0.0 - 1.0)
     * @return The distributor
     */
    public static <T> Distributor<T> makePercent(double percent) {
      return (global, internal) -> (long) (global.getLimit() * percent);
    }

    /**
     * Sets the limits to the global limit minus the number of internal limits * available This
     * allows a single internal limit to take up lots of global limits, leaving <available> for all
     * the other limites If multiple internal limits use more than <available>, than this doesn't
     * guarantee all internal limits will be able to run
     *
     * @param available The number of slots allocated to the internal limits if one limit is using
     *     the rest
     * @return The distributor
     */
    public static Distributor<?> makeLeaveAvailable(long available) {
      return (global, internal) -> {
        long limit = global.getLimit() - internal.size() * available;
        internal.forEach((key, internalLimit) -> internalLimit.setLimit(limit));
        return limit;
      };
    }

    /**
     * Re-distributes the internal rate limits
     *
     * @param globalRateLimit The global rate limit (RateLimit#setLimit not recommended)
     * @param internalLimits The internal limits (RateLimit#setLimit is allowed)
     * @return The limit of the new internal rate limit
     */
    public long distribute(RateLimit globalRateLimit, Map<T, RateLimit> internalLimits);
  }

  private final RateLimit globalRateLimit;
  private final Map<T, RateLimit> internalLimits;
  private final Map<T, RateLimit> internalLimitsView;
  private final Distributor<T> distributor;

  public RateLimitGroup(long limit, long time, Distributor<T> distributor) {
    this.globalRateLimit = new RateLimit(limit, time);
    this.internalLimits = new HashMap<>();
    this.internalLimitsView = Collections.unmodifiableMap(internalLimits);
    this.distributor = distributor;
  }

  public long getLimit() {
    return globalRateLimit.getLimit();
  }

  public long getTime() {
    return globalRateLimit.getTime();
  }

  public boolean exec(T key) {
    if (!globalRateLimit.exec()) {
      return false;
    }
    return internalLimits
        .computeIfAbsent(
            key,
            key2 ->
                new RateLimit(
                    distributor.distribute(globalRateLimit, internalLimitsView),
                    globalRateLimit.getTime()))
        .exec();
  }
}
