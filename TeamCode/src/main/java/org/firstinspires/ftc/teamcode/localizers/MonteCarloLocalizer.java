package org.firstinspires.ftc.teamcode.localizers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorPlus;
import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;

@Config
public class MonteCarloLocalizer extends Localizer {
    // pointMap and monte carlo stuff
    private ArrayList<PoseAndError> pointMap = new ArrayList<PoseAndError>();
    private final DistanceSensorPlus[] sensors;

    private static final int mapSize = 500;
    private static final int selectionSize = (int)(.9 * mapSize);
    private static final int randomPointCount = 5;

    private static double velocityError = .02; // % velocity that drifts due to error

    // synchronization stuff
    private final Object poseLock = new Object(); // lock for pose synchronization across threads
    private final Object mapLock = new Object(); // lock for map synchronization across threads

    private PoseAndError bestPose;

    private final Localizer velocProvider;

    private static final double SECONDS_PER_NANOSECOND = 1e-9;
    private long previousTime = 0;

    public MonteCarloLocalizer(Localizer velocProvider, DistanceSensorPlus... sensors) {
        this.velocProvider = velocProvider;
        velocProvider.shutdown(); // ensure velocProvider is shutdown, so it runs synchronously with this thread

        this.sensors = sensors.clone();

        bestPose = new PoseAndError(new Pose2D(0, 0, 0), 0);
        //setStartPose(new Pose2D(0, 0, 0));
    }

    @Override
    public Pose2D getRobotPose() {
        synchronized (poseLock) {
            return bestPose.pose.copy();
        }
    }

    @Override
    public Pose2D getRobotVelocity() {
        return this.velocProvider.getRobotVelocity();
    }

    public double getPoseError() {
        return bestPose.error;
    }

    @Override
    public void updatePose() {

        if(previousTime == 0) {
            previousTime = System.nanoTime();
        }
        long timeLong = System.nanoTime();
        double dt = (timeLong - previousTime) * SECONDS_PER_NANOSECOND;
        previousTime = timeLong;

        velocProvider.updatePose();
        double heading = velocProvider.getRobotPose().getTheta();

        synchronized (mapLock) {
            if (pointMap.size() < selectionSize) {
                // add a completely uniform distribution of points across the 2/3 field
                // TODO: make gaussian
                for (int i = 0; i < mapSize; i++) {
                    pointMap.add(new PoseAndError(new Pose2D((FieldConstants.CenterToLeftWallWidth_M + FieldConstants.CenterToWallWidth_M) * Math.random() - FieldConstants.CenterToLeftWallWidth_M, (Math.random() - .5) * FieldConstants.WallToWallWidth_M, heading), 0));
                }
            }

            // read actual sensor values
            double[] sensorReadings = new double[sensors.length];
            for (int i = 0; i < sensorReadings.length; i++) {
                sensorReadings[i] = Math.min(sensors[i].sensor.getDistance(DistanceUnit.METER), 2);
            }

            // compute in parallel. Much faster, and means we can have more sample points
            pointMap.parallelStream().forEach(point -> {
                point.error = 0.0;
                for (int j = 0; j < sensorReadings.length; j++) {
                    point.error += Math.abs(sensors[j].simulateSensorReadingAtPose(point.pose) - sensorReadings[j]); // calculate the error between simulated and actual sensor readings
                }
            });
            Collections.sort(pointMap); // sort from lowest error to highest error

            pointMap = new ArrayList<PoseAndError>(pointMap.subList(0, selectionSize));

            // add some completely random points for robustness against great noise
            for (int i = 0; i < randomPointCount; i++) {
                pointMap.add(new PoseAndError(new Pose2D((FieldConstants.CenterToLeftWallWidth_M + FieldConstants.CenterToWallWidth_M) * Math.random() - FieldConstants.CenterToLeftWallWidth_M, (Math.random() - .5) * FieldConstants.WallToWallWidth_M, heading), 0));
            }

            Pose2D veloc_dt = velocProvider.getRobotVelocity();
            veloc_dt.mult(dt);

            // move each point according to robot velocity
            pointMap.parallelStream().forEach(p -> {
                p.pose.add(veloc_dt);
                p.pose.setTheta(heading);
            });
            synchronized (poseLock) {
                bestPose = pointMap.get(0); // grab the point with lowest error
            }

            ArrayList<PoseAndError> newPredictions = new ArrayList<PoseAndError>(mapSize - pointMap.size() + 1);
            for (int i = pointMap.size(); i < mapSize; i++) {
                PoseAndError randomPose = new PoseAndError(pointMap.get((int) (Math.random() * (pointMap.size()))).pose.copy(), 0);
                randomPose.pose.add(2.0 * (Math.random() - .5) * velocityError, 2.0 * (Math.random() - .5) * velocityError, heading);
                newPredictions.add(randomPose);
            }
            pointMap.addAll(newPredictions);
        }
    }

    @Override
    public void setStartPose(Pose2D pose) {
        velocProvider.setStartPose(pose);

        // when setting the startPose, it is convenient to concentrate some of the point map around
        // that point to shorten the time it takes to localize the robot when it first starts
        synchronized (mapLock) {
            pointMap.clear();
            for(int i = 0; i < selectionSize-1; i++) {
                //pointMap.add(new PoseAndError(pose, 0));
                pointMap.add(new PoseAndError(new Pose2D((Math.random() - .5) * FieldConstants.WallToWallWidth_M, (Math.random() - .5) * FieldConstants.WallToWallWidth_M, pose.getTheta()), 0));
            }
        }
    }

    /**
     * directly overrides the internal pointArray
     * use carefully, as this can cause an error if the provided pointArray has less elements than the selection size
     * @param pointArray override list
     */
    public void overridePointMap(ArrayList<Pose2D> pointArray) {
        synchronized (mapLock) {
            pointMap.clear();
            for (Pose2D p : pointArray) {
                pointMap.add(new PoseAndError(p, 0));
            }
        }
    }

    public static class PoseAndError implements Comparable<PoseAndError> {
        public Double error;
        public Pose2D pose;

        public PoseAndError(Pose2D pose, double error) {
            this.pose = pose;
            this.error = error;
        }

        @Override
        public int compareTo(PoseAndError poseAndError) {
            return this.error.compareTo(poseAndError.error);
        }
    }
}
