/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot;

 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 
 import static frc.robot.constants.Constants.VisionConstants.*;
 
 import java.util.List;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.targeting.PhotonPipelineResult;
 import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class Vision {
     private final PhotonCamera camera;
     private final PhotonPoseEstimator photonEstimator;
     PhotonPipelineResult result;
     PhotonTrackedTarget bestTarget;
     int targetId;
     Pose2d pose;
     
     private Matrix<N3, N1> curStdDevs;
 
     public Vision() {
         camera = new PhotonCamera(kCameraName);
 
         photonEstimator =
                 new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
         result = camera.getLatestResult();
         bestTarget = result.getBestTarget();
         targetId = bestTarget.getFiducialId();
     }
 
     public Pose2d decidePoseAlignmentLeft() {
         Pose2d leftSidePoses[] = {
             // TODO: Find the correct poses for the left sides of the reef and feeding stations
             // Poses start from the front face and go clockwise around the reef
             new Pose2d(1.0, 0.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 1.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 0.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 1.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 2.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 3.5, Rotation2d.fromDegrees(0)),
             // Last two are left and right feeding stations
             new Pose2d(1.0, 3.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 3.5, Rotation2d.fromDegrees(0))
         };
 
         // Determine the target position based on the target ID
         if (targetId == 7 || targetId == 18) {
             pose = leftSidePoses[0];
         } else if (targetId == 6 || targetId == 19) {
             pose = leftSidePoses[1];
         } else if (targetId == 11 || targetId == 20) {
             pose = leftSidePoses[2];
         } else if (targetId == 10 || targetId == 21) {
             pose = leftSidePoses[3];
         } else if (targetId == 9 || targetId == 22) {
             pose = leftSidePoses[4];
         } else if (targetId == 8 || targetId == 17) {
             pose = leftSidePoses[5];
         } else if (targetId == 1 || targetId == 13) {
             pose = leftSidePoses[6];
         } else if (targetId == 2 || targetId == 12) {
             pose = leftSidePoses[7];
         } else {
             pose = null;
         }
 
         return pose;
     }
 
     public Pose2d decidePoseAlignmentRight() {
         Pose2d rightSidePoses[] = {
             // TODO: Find the correct poses for the right sides of the reef and feeding stations
             // Poses start from the front face and go clockwise around the reef
             new Pose2d(1.0, 0.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 1.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 0.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 1.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 2.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 3.5, Rotation2d.fromDegrees(0)),
             // Last two are left and right of the feeding station
             new Pose2d(1.0, 3.5, Rotation2d.fromDegrees(0)),
             new Pose2d(1.0, 3.5, Rotation2d.fromDegrees(0))
         };
 
         // Determine the target position based on the target ID
         if (targetId == 7 || targetId == 18) {
             pose = rightSidePoses[0];
         } else if (targetId == 6 || targetId == 19) {
             pose = rightSidePoses[1];
         } else if (targetId == 11 || targetId == 20) {
             pose = rightSidePoses[2];
         } else if (targetId == 10 || targetId == 21) {
             pose = rightSidePoses[3];
         } else if (targetId == 9 || targetId == 22) {
             pose = rightSidePoses[4];
         } else if (targetId == 8 || targetId == 17) {
             pose = rightSidePoses[5];
         } else if (targetId == 1 || targetId == 13) {
             pose = rightSidePoses[6];
         } else if (targetId == 2 || targetId == 12) {
             pose = rightSidePoses[7];
         } else {
             pose = null;
         }
 
         return pose;
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         for (var change : camera.getAllUnreadResults()) {
             visionEst = photonEstimator.update(change);
             updateEstimationStdDevs(visionEst, change.getTargets());
         }
         return visionEst;
     }
 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
     private void updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = kSingleTagStdDevs;
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 curStdDevs = kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs = estStdDevs;
             }
         }
     }
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }
 }
 