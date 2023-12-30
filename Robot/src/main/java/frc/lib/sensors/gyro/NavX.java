package frc.lib.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.sensors.gyro.AHRSProtocol.AHRSUpdateBase;

/** Driver for a NavX board. Basically a wrapper for the {@link AHRS} class */
public class NavX {
  protected class Callback implements ITimestampedDataSubscriber {
    public double getDegreesPerSecond(
        long curTimestamp, double prevMeasurement, double curMeasurement) {
      return 1000.0
          * (-prevMeasurement - curMeasurement)
          / (double) (curTimestamp - mLastSensorTimestampMs);
    }

    @Override
    public void timestampedDataReceived(
        long system_timestamp, long sensor_timestamp, AHRSUpdateBase update, Object context) {
      synchronized (NavX.this) {
        // This handles the fact that the sensor is inverted from our coordinate conventions.
        if (mLastSensorTimestampMs != kInvalidTimestamp
            && mLastSensorTimestampMs < sensor_timestamp) {
          mYawRateDegreesPerSecond = getDegreesPerSecond(sensor_timestamp, mYawDegrees, update.yaw);
          mPitchRateDegreesPerSecond =
              getDegreesPerSecond(sensor_timestamp, mPitchDegrees, update.pitch);
          mRollRateDegreesPerSecond =
              getDegreesPerSecond(sensor_timestamp, mRollDegrees, update.roll);
        }
        mLastSensorTimestampMs = sensor_timestamp;
        mYawDegrees = -update.yaw;
        mPitchDegrees = -update.pitch;
        mRollDegrees = -update.roll;
      }
    }
  }

  protected AHRS mAHRS;
  protected Rotation2d mYawAdjustment = new Rotation2d();
  protected Rotation2d mPitchAdjustment = new Rotation2d();
  protected Rotation2d mRollAdjustment = new Rotation2d();
  protected double mYawDegrees;
  protected double mYawRateDegreesPerSecond;
  protected double mPitchDegrees;
  protected double mPitchRateDegreesPerSecond;
  protected double mRollDegrees;
  protected double mRollRateDegreesPerSecond;
  protected static final long kInvalidTimestamp = -1;
  protected long mLastSensorTimestampMs;

  public NavX(SPI.Port spi_port_id) {
    mAHRS = new AHRS(spi_port_id, (byte) 200);
    resetState();
    mAHRS.registerCallback(new Callback(), null);
    mAHRS.enableBoardlevelYawReset(true);
  }

  public synchronized void reset() {
    mAHRS.reset();
    resetState();
  }

  public boolean isCalibrating() {
    return mAHRS.isCalibrating();
  }

  public synchronized void zeroYaw() {
    mAHRS.zeroYaw();
    resetState();
  }

  private void resetState() {
    mLastSensorTimestampMs = kInvalidTimestamp;
    mYawDegrees = 0.0;
    mYawRateDegreesPerSecond = 0.0;
    mPitchDegrees = 0.0;
    mPitchRateDegreesPerSecond = 0.0;
    mRollDegrees = 0.0;
    mRollRateDegreesPerSecond = 0.0;
  }

  public synchronized void setYawAdjustment(Rotation2d adjustment) {
    mYawAdjustment = adjustment;
  }

  public synchronized void setPitchAdjustment(Rotation2d adjustment) {
    mPitchAdjustment = adjustment;
  }

  public synchronized void setRollAdjustment(Rotation2d adjustment) {
    mRollAdjustment = adjustment;
  }

  protected synchronized double getRawYawDegrees() {
    return mYawDegrees;
  }

  protected synchronized double getRawPitchDegrees() {
    return mPitchDegrees;
  }

  protected synchronized double getRawRollDegrees() {
    return mRollDegrees;
  }

  public Rotation2d getYaw() {
    return mYawAdjustment.rotateBy(Rotation2d.fromDegrees(getRawYawDegrees()));
  }

  public double getYawRateDegreesPerSec() {
    return mYawRateDegreesPerSecond;
  }

  public double getYawRateRadiansPerSec() {
    return Math.toRadians(getYawRateDegreesPerSec());
  }

  public Rotation2d getPitch() {
    return mPitchAdjustment.rotateBy(Rotation2d.fromDegrees(getRawPitchDegrees()));
  }

  public double getPitchRateDegreesPerSec() {
    return mPitchRateDegreesPerSecond;
  }

  public Rotation2d getRoll() {
    return mRollAdjustment.rotateBy(Rotation2d.fromDegrees(getRawRollDegrees()));
  }

  public double getRollRateDegreesPerSec() {
    return mRollRateDegreesPerSecond;
  }

  public double getRawAccelX() {
    return mAHRS.getRawAccelX();
  }

  public synchronized double lastTimestampSeconds() {
    return mLastSensorTimestampMs / 1000;
  }
}
