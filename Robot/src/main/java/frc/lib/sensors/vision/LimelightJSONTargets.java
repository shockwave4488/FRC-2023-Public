package frc.lib.sensors.vision;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;

public final class LimelightJSONTargets {
  private LimelightJSONTargets() {}

  static class LimelightTarget_Generic {
    @JsonProperty("t6c_ts")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    double[] cameraPose_TargetSpace = new double[6];

    @JsonProperty("t6r_fs")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    double[] robotPose_FieldSpace = new double[6];

    @JsonProperty("t6r_ts")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    double[] robotPose_TargetSpace = new double[6];

    @JsonProperty("t6t_cs")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    double[] targetPose_CameraSpace = new double[6];

    @JsonProperty("t6t_rs")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    double[] targetPose_RobotSpace = new double[6];

    @JsonProperty("ta")
    double ta;

    @JsonProperty("tx")
    double tx;

    @JsonProperty("txp")
    double tx_pixels;

    @JsonProperty("ty")
    double ty;

    @JsonProperty("typ")
    double ty_pixels;

    @JsonProperty("ts")
    double ts;
  }

  static class LimelightTarget_Retro extends LimelightTarget_Generic {}

  static class LimelightTarget_Fiducial extends LimelightTarget_Generic {
    @JsonProperty("fID")
    double fiducialID;

    @JsonProperty("fam")
    String fiducialFamily;
  }

  static class LimelightTarget_Barcode {}

  static class LimelightTarget_Classifier {
    @JsonProperty("class")
    String className;

    @JsonProperty("classID")
    double classID;

    @JsonProperty("conf")
    double confidence;

    @JsonProperty("zone")
    double zone;

    @JsonProperty("tx")
    double tx;

    @JsonProperty("txp")
    double tx_pixels;

    @JsonProperty("ty")
    double ty;

    @JsonProperty("typ")
    double ty_pixels;
  }

  static class LimelightTarget_Detector {
    @JsonProperty("class")
    String className;

    @JsonProperty("classID")
    double classID;

    @JsonProperty("conf")
    double confidence;

    @JsonProperty("ta")
    double ta;

    @JsonProperty("tx")
    double tx;

    @JsonProperty("txp")
    double tx_pixels;

    @JsonProperty("ty")
    double ty;

    @JsonProperty("typ")
    double ty_pixels;
  }

  static class Results {
    @JsonProperty("pID")
    double pipelineID;

    @JsonProperty("tl")
    double latency_pipeline;

    @JsonProperty("tl_cap")
    double latency_capture;

    @SuppressFBWarnings("URF_UNREAD_FIELD")
    double latency_jsonParse;

    @JsonProperty("ts")
    double timestamp_LIMELIGHT_publish;

    @JsonProperty("ts_rio")
    double timestamp_RIOFPGA_capture;

    @JsonProperty("v")
    @JsonFormat(shape = Shape.NUMBER)
    boolean valid;

    @JsonProperty("botpose")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    double[] botpose = new double[6];

    @JsonProperty("botpose_wpired")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    double[] botpose_wpired = new double[6];

    @JsonProperty("botpose_wpiblue")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    double[] botpose_wpiblue = new double[6];

    @JsonProperty("Retro")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    LimelightTarget_Retro[] targets_Retro = new LimelightTarget_Retro[0];

    @JsonProperty("Fiducial")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    LimelightTarget_Fiducial[] targets_Fiducials = new LimelightTarget_Fiducial[0];

    @JsonProperty("Classifier")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    LimelightTarget_Classifier[] targets_Classifier = new LimelightTarget_Classifier[0];

    @JsonProperty("Detector")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    LimelightTarget_Detector[] targets_Detector = new LimelightTarget_Detector[0];

    @JsonProperty("Barcode")
    @SuppressFBWarnings("URF_UNREAD_FIELD")
    LimelightTarget_Barcode[] targets_Barcode = new LimelightTarget_Barcode[0];
  }

  static class LimelightResults {
    @JsonProperty("Results")
    Results targetingResults;

    LimelightResults() {
      targetingResults = new Results();
    }
  }
}
