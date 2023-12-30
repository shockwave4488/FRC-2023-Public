package frc.lib.sensors.vision;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;

@SuppressFBWarnings("URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD")
public class LimelightJSON {

  // SpotBugs says that the fields aren't getting set
  // This causes NP_UNWRITTEN_PUBLIC_OR_PROTECTED_FIELD
  // Since this error is outside this class, it can't be easily suppressed
  // So these methods trick it into thinking it has a value
  private static <T> T nothing() {
    return null;
  }

  private static byte nothingNum() {
    return 0;
  }

  @SuppressFBWarnings("URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD")
  public static class Result {
    public double[][] pts = nothing();
    public double[] t6c_ts = nothing();
    public double[] t6r_fs = nothing();
    public double[] t6r_ts = nothing();
    public double[] t6t_cs = nothing();
    public double[] t6t_rs = nothing();
    public double ta = nothingNum();
    public double tx = nothingNum();
    public double txp = nothingNum();
    public double ty = nothingNum();
    public double typ = nothingNum();
  }

  public static class RetroResult extends Result {}

  @SuppressFBWarnings("URF_UNREAD_PUBLIC_OR_PROTECTED_FIELD")
  public static class FiducialResult extends Result {
    public int fID = nothingNum();
    public String fam = nothing();
    public double[] skew = nothing();
  }

  public int pID = nothingNum();
  public double tl = nothingNum();
  public double ts = nothingNum();
  public int v = nothingNum();
  public double[] botpose = nothing();

  public RetroResult[] Retro = nothing();
  public FiducialResult[] Fiducial = nothing();
  public Object[] Detector = nothing();
  public Object[] Classifier = nothing();
}
