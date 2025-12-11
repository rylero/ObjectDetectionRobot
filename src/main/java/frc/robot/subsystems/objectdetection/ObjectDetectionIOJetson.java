package frc.robot.subsystems.objectdetection;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class ObjectDetectionIOJetson implements ObjectDetectionIO {
  private NetworkTable jetsonTable;

  // Camera Connectivity Data
  private IntegerSubscriber heartbeat;
  private double heartbeatChangeTime;
  private double prevHeartbeat;

  // Target Observation Data
  private DoubleArraySubscriber targetDXs;
  private DoubleArraySubscriber targetDYs;
  private DoubleArraySubscriber targetAreas;
  private DoubleArraySubscriber targetConfidences;
  private DoubleArraySubscriber targetTimestamps;

  public ObjectDetectionIOJetson() {
    jetsonTable = NetworkTableInstance.getDefault().getTable(ObjectDetectionConstants.JetsonTable);

    heartbeat =
        jetsonTable
            .getIntegerTopic("heartbeat")
            .subscribe(ObjectDetectionConstants.UNCONNECTED_HEARTBEAT_VALUE);

    targetDXs = jetsonTable.getDoubleArrayTopic("targetDX").subscribe(new double[0]);
    targetDYs = jetsonTable.getDoubleArrayTopic("targetDY").subscribe(new double[0]);
    targetAreas = jetsonTable.getDoubleArrayTopic("targetArea").subscribe(new double[0]);
    targetConfidences =
        jetsonTable.getDoubleArrayTopic("targetConfidence").subscribe(new double[0]);
    targetTimestamps = jetsonTable.getDoubleArrayTopic("targetTimestamps").subscribe(new double[0]);
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.connected = isConnected();

    double[] currentTargetDXs = targetDXs.get();
    double[] currentTargetDYs = targetDYs.get();
    double[] currentTargetAreas = targetAreas.get();
    double[] currentTargetConfidences = targetConfidences.get();
    double[] currentTargetTimestmps = targetTimestamps.get();

    if (currentTargetDXs.length != currentTargetDYs.length
        || currentTargetDYs.length != currentTargetAreas.length
        || currentTargetAreas.length != currentTargetConfidences.length
        || currentTargetConfidences.length != currentTargetTimestmps.length) {
      DataLogManager.log("ERROR PROCESSING OBJECT DETECTION: Unequal detection lengths");
      inputs.targetObservations = new TargetObservation[0];
      return;
    }

    inputs.targetObservations = new TargetObservation[currentTargetDXs.length];
    for (int i = 0; i < currentTargetDXs.length; i++) {
      inputs.targetObservations[i] =
          new TargetObservation(
              currentTargetDXs[i],
              currentTargetDYs[i],
              currentTargetAreas[i],
              currentTargetConfidences[i],
              currentTargetTimestmps[i]);
    }
  }

  private boolean isConnected() {
    double cur = heartbeat.get();
    double now = Timer.getFPGATimestamp();

    if (cur <= ObjectDetectionConstants.UNCONNECTED_HEARTBEAT_VALUE) {
      return false;
    }

    if (cur != prevHeartbeat) {
      prevHeartbeat = cur;
      heartbeatChangeTime = now;
    }

    return (now - heartbeatChangeTime) < ObjectDetectionConstants.HEARTBEAT_TOLERANCE;
  }
}
