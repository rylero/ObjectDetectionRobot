package frc.robot.subsystems.objectdetection;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.objectdetection.ObjectDetectionIO.TargetObservation;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.DoubleFunction;

import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {
	private final ObjectDetectionIO io;
	private final ObjectDetectionIOInputsAutoLogged inputs;
	private final DoubleFunction<Optional<Pose2d>> timestampPoseFunction;

	private ArrayList<TrackedObject> trackedObjects;

	public ObjectDetection(
			ObjectDetectionIO io, DoubleFunction<Optional<Pose2d>> timestampPoseFunction) {
		this.io = io;
		inputs = new ObjectDetectionIOInputsAutoLogged();

		trackedObjects = new ArrayList<TrackedObject>();
		this.timestampPoseFunction = timestampPoseFunction;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("ObjectDetection", inputs);

		int i = 0;
		for (TargetObservation target : inputs.targetObservations) {
			Optional<Pose2d> targetPose = getObjectWorldPose(target);
			if (targetPose.isEmpty()) {
				continue;
			}
			trackedObjects.get(i).update(targetPose.get(), target.timestamp());
			i++;
		}

		Pose2d[] trackedObjectPoses = trackedObjects.stream()
			.map(TrackedObject::getPose)
			.toArray(Pose2d[]::new);
			
		Logger.recordOutput("ObjectDetection/ObjectPoses", trackedObjectPoses);
	}

	public TrackedObject[] getTrackedObjects() {
		return trackedObjects.toArray(TrackedObject[]::new);
	}

	public Pose2d[] getTrackedObjectPoses() {
		return trackedObjects.stream()
			.map(TrackedObject::getPose)
			.toArray(Pose2d[]::new);
	}

	public Optional<Pose2d> getObjectWorldPose(TargetObservation observation) {
		Optional<Pose2d> robotPose2d = timestampPoseFunction.apply(observation.timestamp());
		if (robotPose2d.isEmpty()) {
			return Optional.empty();
		}

		Pose2d transformedPose = robotPose2d
				.get()
				.transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA)
				.transformBy(
						new Transform2d(
								new Translation2d(observation.dx(), observation.dy()), new Rotation2d(0.0)));

		return Optional.of(transformedPose);
	}

	public class TrackedObject {
		private Pose2d filteredPose;
		private double lastSeenTimestamp;

		private LinearFilter poseXFilter = LinearFilter.movingAverage(ObjectDetectionConstants.POSE_FILTER_TAPS);
		private LinearFilter poseYFilter = LinearFilter.movingAverage(ObjectDetectionConstants.POSE_FILTER_TAPS);

		public TrackedObject(Pose2d pose, double timestamp) {
			lastSeenTimestamp = timestamp;

			filteredPose = new Pose2d(
				poseXFilter.calculate(pose.getX()),
				poseYFilter.calculate(pose.getY()),
				Rotation2d.kZero
			);
		}

		public void update(Pose2d newPose, double newTimestamp) {
			lastSeenTimestamp = newTimestamp;

			filteredPose = new Pose2d(
				poseXFilter.calculate(newPose.getX()),
				poseYFilter.calculate(newPose.getY()),
				Rotation2d.kZero
			);
		}

		public Pose2d getPose() {
			return filteredPose;
		}

		public double getLastSeenTimestamp() {
			return lastSeenTimestamp;
		}
	}
}
