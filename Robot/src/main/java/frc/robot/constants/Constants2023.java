package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.misc.CollectionUtil;
import frc.lib.misc.MatchUtil;
import frc.robot.constants.Constants2023.RobotConstants.ArmConstants;
import frc.robot.constants.Constants2023.RobotConstants.ArmConstants.ArmSetpoint;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collector;
import java.util.stream.Stream;

public final class Constants2023 extends Constants {
  public enum GamePiece {
    Cube(Units.inchesToMeters(-3)),
    Cone(Units.inchesToMeters(0));

    private final double intakeOffset;

    private GamePiece(double intakeOffset) {
      this.intakeOffset = intakeOffset;
    }

    public double getIntakeOffset() {
      return intakeOffset;
    }
  }

  public enum ScoreLevel {
    HIGH(FieldConstants.gridRowX.get(3), ArmSetpoint.HIGH_SCORE),
    MID(FieldConstants.gridRowX.get(2), ArmSetpoint.MID_SCORE),
    LOW(FieldConstants.gridRowX.get(1), ArmSetpoint.LOW_SCORE);

    private final double x;
    private final ArmSetpoint armValue;

    private ScoreLevel(double x, ArmSetpoint armValue) {
      this.x = x;
      this.armValue = armValue;
    }

    public double getX() {
      return x;
    }

    public ArmSetpoint getArmSetpoint() {
      return armValue;
    }

    public double getOffsetFromHigh() {
      double armOffset =
          ArmConstants.ARM_LENGTH
              * (Math.cos(HIGH.armValue.angleRadians) - Math.cos(armValue.angleRadians));
      return x - HIGH.x + armOffset;
    }
  }

  public enum ScorePosition {
    CONE_LEFT(GamePiece.Cone, -FieldConstants.INNER_HYBRID_NODE_DIFF),
    CUBE_CENTER(GamePiece.Cube, 0),
    CONE_RIGHT(GamePiece.Cone, FieldConstants.INNER_HYBRID_NODE_DIFF);

    private final GamePiece piece;
    private final double offset;

    private ScorePosition(GamePiece piece, double offset) {
      this.piece = piece;
      this.offset = offset;
    }

    public GamePiece getPiece() {
      return piece;
    }

    public double getOffset() {
      return offset;
    }
  }

  public enum DoubleSubstationSide {
    Left(FieldConstants.tagToEndOfSubstation / 2),
    Right(-FieldConstants.tagToEndOfSubstation / 2);

    public final double offsetToSideCenter;

    DoubleSubstationSide(double offsetToSideCenter) {
      this.offsetToSideCenter = offsetToSideCenter;
    }
  }

  public static record Node(int row, int col) implements Comparable<Node> {
    public enum Type {
      Shelf,
      Pole,
      Hybrid
    }

    /** Row goes bottom to top, column goes left to right. */
    public Node {
      if (row < 1 || row > 3) {
        throw new IllegalArgumentException("Node row must be between 1 and 3.");
      }
      if (col < 1 || col > 9) {
        throw new IllegalArgumentException("Node column must be between 1 and 9.");
      }
    }

    public Translation2d getPosition() {
      return FieldConstants.getInstance().gridNodePositions.get(this);
    }

    public Type getType() {
      return FieldConstants.nodeTypes.get(this);
    }

    @Override
    public int compareTo(Node node) {
      return (int) Math.signum(col - node.col);
    }

    @Override
    public String toString() {
      return String.format("Node: row = %d, col = %d", row, col);
    }
  }

  public static final class FieldConstants {
    private static FieldConstants instance;
    private final Alliance alliance = DriverStation.getAlliance();

    public static final double FIELD_LENGTH = 16.54175;
    public static final double FIELD_WIDTH = 8.001;

    public static Translation2d toRedAlliance(Translation2d translation) {
      // For relative positions, X value doesn't need to be flipped because of y-axis symmetry
      return new Translation2d(translation.getX(), FIELD_WIDTH - translation.getY());
    }

    public static Translation2d toRedAllianceAbsolute(Translation2d translation) {
      return new Translation2d(FIELD_LENGTH - translation.getX(), FIELD_WIDTH - translation.getY());
    }

    /**
     * @param isRelative Whether the pose should be transformed taking advantage of the symmetry of
     *     the field. If this is true, a given field location (node, charge station, etc.) on the
     *     blue alliance side will be turned into the pose of the corresponding red field element.
     */
    public static Pose3d toRedAlliance(Pose3d pose, boolean isRelative) {
      Function<Translation2d, Translation2d> alliancePosSwitch =
          isRelative ? FieldConstants::toRedAlliance : FieldConstants::toRedAllianceAbsolute;
      Translation2d translation2d =
          alliancePosSwitch.apply(pose.getTranslation().toTranslation2d());
      Rotation3d curRotation = pose.getRotation();
      Rotation3d newRotation =
          !isRelative
              ? new Rotation3d(
                  curRotation.getX(),
                  curRotation.getY(),
                  curRotation.toRotation2d().rotateBy(Rotation2d.fromRotations(0.5)).getRadians())
              : curRotation;
      return new Pose3d(
          new Translation3d(
              translation2d.getX(), translation2d.getY(), pose.getTranslation().getZ()),
          newRotation);
    }

    public static final Map<Alliance, List<Integer>> allianceCubeScoringTagIds =
        Collections.unmodifiableMap(
            new EnumMap<>(Map.of(Alliance.Red, List.of(1, 2, 3), Alliance.Blue, List.of(6, 7, 8))));
    public static final List<Integer> cubeScoringTagIds =
        Stream.concat(
                allianceCubeScoringTagIds.get(Alliance.Red).stream(),
                allianceCubeScoringTagIds.get(Alliance.Blue).stream())
            .toList();

    public static final Map<Alliance, Integer> substationTagIds =
        Collections.unmodifiableMap(new EnumMap<>(Map.of(Alliance.Blue, 4, Alliance.Red, 5)));
    public final int substationTagId = substationTagIds.get(alliance);

    public static final double tagToEndOfSubstation = Units.feetToMeters(4);

    public static final List<Integer> poleColumns = List.of(1, 3, 4, 6, 7, 9);
    public static final List<Integer> shelfColumns = List.of(2, 5, 8);
    public static final Map<Node, Node.Type> nodeTypes;

    public static synchronized FieldConstants getInstance() {
      if (instance == null) {
        if (MatchUtil.isRealMatch().isEmpty() && DriverStation.isDisabled()) {
          throw new RuntimeException(
              """
              No FieldConstants instance available at the time of calling getInstance(). \
              Most likely the call should be moved to a place that will only be run \
              when the robot is enabled.""");
        } else {
          instance = new FieldConstants();
        }
      }
      return instance;
    }

    public static void eagerlyInitialize() {
      // Either enabled or connected to the FMS at this point
      MatchUtil.runOnRealMatchDetermination(
          () -> {
            synchronized (FieldConstants.class) {
              if (instance == null) instance = new FieldConstants();
            }
          });
    }

    static {
      final Map<Node, Node.Type> nodeTypeMap = new HashMap<>();
      for (int row = 1; row <= 3; row++) {
        for (int col = 1; col <= 9; col++) {
          Node.Type nodeType;
          if (row == 1) {
            nodeType = Node.Type.Hybrid;
          } else {
            nodeType = (shelfColumns.contains(col)) ? Node.Type.Shelf : Node.Type.Pole;
          }
          nodeTypeMap.put(new Node(row, col), nodeType);
        }
      }
      nodeTypes = Collections.unmodifiableMap(nodeTypeMap);
    }

    /**
     * Returns a new value after inputting the {@code val} into the {@code modifier} if on the red
     * alliance, else returns the current value.
     */
    public <T> T modifyIfRed(T val, Function<T, T> modifier) {
      if (alliance == Alliance.Red) {
        return modifier.apply(val);
      }
      return val;
    }

    /**
     * Collects the stream of values from {@code collection} after modifying them if on the red
     * alliance.
     */
    public <T, S> S modifyEachIfRed(
        Collection<T> collection, Function<T, T> modifier, Collector<T, ?, S> collector) {
      return collection.stream().map(val -> modifyIfRed(val, modifier)).collect(collector);
    }

    public final AprilTagFieldLayout tagsOnField =
        new AprilTagFieldLayout(
            List.of(
                new AprilTag(1, new Pose3d(15.5136, 1.0716, 0.4628, new Rotation3d(0, 0, Math.PI))),
                new AprilTag(2, new Pose3d(15.5136, 2.7480, 0.4628, new Rotation3d(0, 0, Math.PI))),
                new AprilTag(3, new Pose3d(15.5136, 4.4244, 0.4628, new Rotation3d(0, 0, Math.PI))),
                new AprilTag(4, new Pose3d(16.1788, 6.7498, 0.6955, new Rotation3d(0, 0, Math.PI))),
                new AprilTag(5, new Pose3d(0.3620, 6.7498, 0.6955, new Rotation3d(0, 0, 0))),
                new AprilTag(6, new Pose3d(1.0274, 4.4244, 0.4628, new Rotation3d(0, 0, 0))),
                new AprilTag(7, new Pose3d(1.0274, 2.7480, 0.4628, new Rotation3d(0, 0, 0))),
                new AprilTag(8, new Pose3d(1.0274, 1.0716, 0.4628, new Rotation3d(0, 0, 0)))),
            FIELD_LENGTH,
            FIELD_WIDTH);

    public static final double CHARGE_STATION_CENTER_X = 3.835;
    public final Translation2d CHARGE_STATION_CENTER =
        modifyIfRed(
            new Translation2d(CHARGE_STATION_CENTER_X, 2.744), FieldConstants::toRedAlliance);

    // Theoretical is 3.5 in., but practice field measurement is 3 1/16 in.
    private static final double NODE_DOUBLE_DIVIDER_WIDTH = Units.inchesToMeters(3.5);
    private static final double OUTER_HYBRID_NODE_WIDTH = Units.inchesToMeters(25.75);
    private static final double INNER_HYBRID_NODE_WIDTH = Units.inchesToMeters(18.5);
    private static final double INNER_HYBRID_NODE_DIFF_OFFSET =
        Units.inchesToMeters(0); // change at comp
    private static final double INNER_HYBRID_NODE_DIFF =
        NODE_DOUBLE_DIVIDER_WIDTH + INNER_HYBRID_NODE_WIDTH + INNER_HYBRID_NODE_DIFF_OFFSET;
    private static final double POLE_TO_OUTER_NODE_CENTER =
        (OUTER_HYBRID_NODE_WIDTH - INNER_HYBRID_NODE_WIDTH) / 2;
    // Lower columns are to the right from the perspective of the drive station
    public final Map<Integer, Double> hybridNodeColumnY;
    public final Map<Integer, Double> specificNodeColumnY;
    // Lower-numbered rows are lower (1 is hybrid, 3 is top)
    // There's an x-value difference of 7/8 in. between the center of a hybrid node on a cube column
    // vs. a cone column. So, the constant here is in between those two.
    public static final Map<Integer, Double> gridRowX = Map.of(1, 1.191, 2, 0.851, 3, 0.419);
    public final Map<Node, Translation2d> gridNodePositions;

    public static final double PRESET_PIECE_X = 7.118;
    public final Map<Integer, Translation2d> presetPiecePositions =
        modifyIfRed(
            Map.of(
                1, new Translation2d(PRESET_PIECE_X, 4.577),
                2, new Translation2d(PRESET_PIECE_X, 3.358),
                3, new Translation2d(PRESET_PIECE_X, 2.138),
                4, new Translation2d(PRESET_PIECE_X, 0.919)),
            pieceMap ->
                CollectionUtil.reverseMap(
                    CollectionUtil.modifyMapValue(pieceMap, FieldConstants::toRedAlliance)));

    public FieldConstants() {
      if (alliance == Alliance.Red) {
        // Unlike the convention, our constants use the current alliance on the right side,
        // because of the default origin position and our blue-relative hard-coded AprilTag poses.
        tagsOnField.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      }

      final Map<Integer, Double> specificNodeColumnYMap = new HashMap<>();
      for (int col = 1; col <= 9; col++) {
        specificNodeColumnYMap.put(
            col,
            NODE_DOUBLE_DIVIDER_WIDTH
                + OUTER_HYBRID_NODE_WIDTH / 2
                + POLE_TO_OUTER_NODE_CENTER
                + INNER_HYBRID_NODE_DIFF * (col - 1));
      }
      specificNodeColumnY = Collections.unmodifiableMap(specificNodeColumnYMap);

      final Map<Integer, Double> hybridNodeColumnYMap = new HashMap<>(specificNodeColumnY);
      hybridNodeColumnYMap.compute(1, (col, y) -> y - POLE_TO_OUTER_NODE_CENTER);
      hybridNodeColumnYMap.compute(9, (col, y) -> y + POLE_TO_OUTER_NODE_CENTER);
      hybridNodeColumnY = Collections.unmodifiableMap(hybridNodeColumnYMap);

      Map<Node, Translation2d> nodePositionMap = new HashMap<>();

      for (int row = 1; row <= 3; row++) {
        Map<Node, Translation2d> nodePositionRowMap = new HashMap<>();
        for (int col = 1; col <= 9; col++) {
          nodePositionRowMap.put(
              new Node(row, col),
              new Translation2d(
                  gridRowX.get(row),
                  (row != 1) ? specificNodeColumnY.get(col) : hybridNodeColumnY.get(col)));
        }
        nodePositionRowMap = modifyIfRed(nodePositionRowMap, CollectionUtil::reverseMap);
        nodePositionMap.putAll(nodePositionRowMap);
      }
      nodePositionMap =
          CollectionUtil.modifyMapValue(
              nodePositionMap,
              translation -> modifyIfRed(translation, FieldConstants::toRedAlliance));
      nodePositionMap =
          CollectionUtil.modifyMapValue(
              nodePositionMap,
              translation -> modifyIfRed(translation, FieldConstants::toRedAllianceAbsolute));
      gridNodePositions = Collections.unmodifiableMap(nodePositionMap);
    }
  }

  public static final class RobotConstants {
    public static final class DriveConstants {
      public static final double SWERVE_HALF_ROTATION_MULTIPLIER = -0.5;
    }

    public static final class IntakeConstants {
      public static final double FLIGHT_SENSORS_OFFSET = 0.1;
      // What is outputted when there isn't a cone (so the distance to the other side of the intake)
      public static final double FLIGHT_SENSORS_MAX_VALUE = 0.47;
      public static final double EXTENSION_LENGTH = Units.inchesToMeters(6);
    }

    public static final class AutonomousConstants {
      public static final double AUTO_MAX_VELOCITY = 3.0;
      public static final double AUTO_MAX_ACCELERATION = 3;
      public static final double AUTO_BALANCE_P = 0.025;
      public static final double AUTO_BALANCE_I = 0;
      public static final double AUTO_BALANCE_D = 0;
    }

    public static final class ArmConstants {
      public enum ArmSetpoint {
        LOW_SCORE(-1.3),
        MID_SCORE(-0.31),
        HIGH_SCORE(0.03),
        SUBSTATION(-0.32),
        PIECE_PICKUP(-1.46),
        DEFAULT_MINIMUM(-1.371);

        public final double angleRadians;

        ArmSetpoint(double angleRadians) {
          this.angleRadians = angleRadians;
        }

        public double getArmOutDistance() {
          return ARM_LENGTH * Math.cos(angleRadians);
        }

        public double getArmOutDistanceWithIntake() {
          return ARM_LENGTH * Math.cos(angleRadians) + IntakeConstants.EXTENSION_LENGTH;
        }
      }

      public static final double ARM_MAXIMUM_VALUE = 0.07;
      public static final double ARM_CHANGE = 0.02;
      public static final int ARM_CURRENT_LIMIT = 40;
      public static final double ARM_LENGTH = Units.inchesToMeters(41.5);
      public static final double ROBOT_CENTER_TO_BEAM = Units.inchesToMeters(11.5);
    }
  }
}
