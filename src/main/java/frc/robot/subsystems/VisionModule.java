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

package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public final class VisionModule implements Constants.PhotonVision, Constants.Swerve {

    private final PhotonCamera aprilTagsFrontRight;
    private final PhotonCamera aprilTagsRearLeft;
    private PhotonPoseEstimator photonEstimatorFrontRight;
    private PhotonPoseEstimator photonEstimatorRearLeft;


    private final PhotonCamera notesIndexer;
    private boolean startTrackingNotes = false;


    private static VisionModule instance;

    private VisionPipelineRunnable notePipelineVisionPoller;

    private VisionModule() {
        aprilTagsFrontRight = new PhotonCamera(APRIL_TAGS_FRONT_RIGHT_CAMERA_NAME);
        aprilTagsRearLeft = new PhotonCamera(APRIL_TAGS_FRONT_RIGHT_CAMERA_NAME);

        photonEstimatorFrontRight =
                new PhotonPoseEstimator(
                        tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagsFrontRight, ROBOT_TO_TAG_FRONT_RIGHT_CAM_POS);
        photonEstimatorFrontRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorRearLeft =
                new PhotonPoseEstimator(
                        tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagsRearLeft, ROBOT_TO_TAG_REAR_LEFT_CAM_POS);
        photonEstimatorRearLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        notesIndexer = new PhotonCamera(NOTES_INDEXER_CAMERA_NAME);


        Thread notePipelineVisionThread = new Thread(new VisionPipelineRunnable(notesIndexer, 90), "notesVisionThread");
        notePipelineVisionThread.setDaemon(true);
        notePipelineVisionThread.start();
    }

    public static synchronized VisionModule getInstance() {
        if (instance == null) {
            instance = new VisionModule();
        }
        return instance;
    }

    public PhotonCamera getAprilTagsFrontRightCamera() {
        return aprilTagsFrontRight;
    }

    public PhotonCamera getAprilTagsRearLeftCamera() {
        return aprilTagsRearLeft;
    }

    public PhotonPoseEstimator getPhotonEstimatorFrontRight() {
        return photonEstimatorFrontRight;
    }

    public PhotonPoseEstimator getPhotonEstimatorRearLeft() {
        return photonEstimatorRearLeft;
    }

    public PhotonCamera getNotesIndexerCamera() {
        return notesIndexer;
    }

    public PhotonPipelineResult getRecentNoteResult() {
        return notePipelineVisionPoller.getRecentResult();
    }


    public boolean isStartTrackingNotes() {
        return startTrackingNotes;
    }

    public void setStartTrackingNotes(boolean startTrackingNotes) {
        this.startTrackingNotes = startTrackingNotes;
        notePipelineVisionPoller.setEnabled(startTrackingNotes);
    }


    /**
     * A Thread Runnable that takes a PhotonCamera and continuously
     * gets its pipeline results (does not select/change pipeline)
     * and keeps the last seen result available, unless it is older
     * that the configured (at instantiation time) TTL.
     *
     * If it's older than the TTL, then will block until there is
     * a result available.
     *
     * The Runnable also tracks the average time to receive a result,
     * and if, based upon that, one is expected to arrive 'soon', it
     * will wait for it.
     */
    public class VisionPipelineRunnable implements Runnable {

        PhotonCamera camera;

        long TTLnanos = 100;

        PhotonPipelineResult lastResultAvailable;
        long lastResultTimestampNanos = 0;
        long avgResultLatency = 0;

        public static final long TWENTY_MILLIS_IN_NANOS = TimeUnit.MILLISECONDS.toNanos(20);

        private int SAMPLE_SIZE = 8;
        long[] latencySamples = new long[SAMPLE_SIZE];
        int lastLatencySampleIndex = -1;

        final AtomicBoolean enabled = new AtomicBoolean();

        public VisionPipelineRunnable(PhotonCamera camera, long TTLinMillis) {
            this.camera = camera;
            this.TTLnanos = TimeUnit.MILLISECONDS.toNanos(TTLinMillis);
        }


        public boolean isEnabled() {
            return enabled.get();
        }

        public void setEnabled(boolean enabled) {
            synchronized (this.enabled) {
                this.enabled.set(enabled);
                this.enabled.notify();
            }
        }

        @Override
        public void run() {

            while(true) {
                synchronized (enabled) {
                    if (!enabled.get()) {
                        try {
                            enabled.wait();
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        continue;
                    }
                }

                // get (wait for) the next result
                long sTime = System.nanoTime();
                PhotonPipelineResult newResult = camera.getLatestResult();
                long eTime = System.nanoTime();

                // calculate new average latency for the past window of time
                lastLatencySampleIndex = (lastLatencySampleIndex++) % SAMPLE_SIZE;
                latencySamples[lastLatencySampleIndex] = eTime - sTime;
                long newAvg = 0;
                int numSamplesToAvg = 0;
                for(; numSamplesToAvg < SAMPLE_SIZE; numSamplesToAvg++) {
                    if(latencySamples[numSamplesToAvg] == 0)
                        break;
                    newAvg += latencySamples[numSamplesToAvg];
                }
                newAvg = newAvg / numSamplesToAvg;

                synchronized (this) {
                    avgResultLatency = newAvg;
                    lastResultTimestampNanos = eTime;
                    lastResultAvailable = newResult;
                    notify();
                }

                // if by slim chance we're going way fast, wait a bit to not overburdden the poor roborio
                if(latencySamples[lastLatencySampleIndex] < TWENTY_MILLIS_IN_NANOS) {
                    try {
                        Thread.sleep(5);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }

        /**
         * @return the most recent pipeline result.  it is unexpected but
         * possible for this method to return null, which must be handled by the caller
         */
        public PhotonPipelineResult getRecentResult()  {

            synchronized (this) {

                long since = System.nanoTime() - lastResultTimestampNanos;

                // if the last result is too old, or if we're within small % of avg latency,
                // just wait for the next result
                if ((since > TTLnanos) || ((since - avgResultLatency) < (avgResultLatency * 0.15)) ){
                    lastResultAvailable = null;
                    try {
                        wait();
                    } catch (InterruptedException e) {
                        return null;
                    }
                }
                return lastResultAvailable;
            }
        }
    }

}

