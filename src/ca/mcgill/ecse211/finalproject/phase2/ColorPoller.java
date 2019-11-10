package ca.mcgill.ecse211.finalproject.phase2;

import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.leftColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.rightColorSensor;
import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
import ca.mcgill.ecse211.finalproject.Resources;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * Class which takes care of color sensor polling, signal filtering as well as odometry correction using the color
 * sensors
 * 
 * @author yp
 *
 */
public class ColorPoller implements Runnable {
  // =====left sensor=====//
  /**
   * sample provider for the left color sensor
   */
  private static SampleProvider leftSampleProvider = leftColorSensor.getRedMode();
  /**
   * sample reading from the left color sensor
   */
  private static float[] leftSampleColor = new float[leftColorSensor.sampleSize()];
  /**
   * value from left sensor
   */
  private static float leftSample;
  /**
   * change in sensor measurement between two iterations of the left color sensor sampling
   */
  private static float leftDer = 0;
  /**
   * used to hold previous value of the left sampler
   */
  private static float leftPrev = 0;

  // =====right sensor=====//
  /**
   * sample provider for the right color sensor
   */
  private static SampleProvider rightSampleProvider = rightColorSensor.getRedMode();
  /**
   * sample reading from the right color sensor
   */
  private static float[] rightSampleColor = new float[rightColorSensor.sampleSize()];
  /**
   * value from right sensor
   */
  private static float rightSample;
  /**
   * change in sensor measurement between two iterations of the right color sensor sampling
   */
  private static volatile float rightDer = 0f;
  /**
   * used to hold previous value of the right sampler
   */
  private static volatile float rightPrev = 0f;

  /**
   * A threshold value used to determine a large enough change in sensor measurement
   */
  private static final float SENSOR_DERIVATIVE_THRESHOLD = 0.05f;

  /**
   * a global kill switch for the poller thread
   */
  public static boolean kill = false;
  /**
   * whether or not the left sensor is currently detecting a black line
   */
  public static volatile boolean leftLineDetected;
  /**
   * whether or not the right sensor is currently detecting a black line
   */
  public static volatile boolean rightLineDetected;
  /**
   * flag indicating whether the robot is in the process of odometry correction
   */
  public static volatile boolean isCorrecting = false;

  public static boolean wait = false;

  private void readLeft() {
    leftSampleProvider.fetchSample(leftSampleColor, 0);
    leftSample = leftSampleColor[0];
    leftDer = leftSample - leftPrev;
  }

  private void readRight() {
    rightSampleProvider.fetchSample(rightSampleColor, 0);
    rightSample = rightSampleColor[0];
    rightDer = rightSample - rightPrev;
  }

  /**
   * run method for the two color poller
   */
  public void run() {
    long readingStart, readingEnd;
    while (!kill) {
      if (wait) {
        try {
          Thread.sleep(60);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      } else {
        readingStart = System.currentTimeMillis();

        if (!leftLineDetected)
          readLeft();
        if (!rightLineDetected)
          readRight();

        synchronized (navigation) {
          if (navigation.navigationMode == TravelingMode.TRAVELING
              || navigation.navigationMode == TravelingMode.OBSTACLE_ENCOUNTERED) {
            leftLineDetected = leftDetectBlackLine();
            rightLineDetected = rightDetectBlackLine();
            if (leftLineDetected || rightLineDetected) {
              navigation.navigationMode = TravelingMode.CORRECTING;
              navigation.setSpeed(40);
              navigation.interrupted = true;
            }
          } else if (navigation.navigationMode == TravelingMode.CORRECTING) {
            if (!leftLineDetected) {
              leftLineDetected = leftDetectBlackLine();
            } else if (!rightLineDetected) {
              rightLineDetected = rightDetectBlackLine();
            }
          }
        }

        leftPrev = leftSample;
        rightPrev = rightSample;
        readingEnd = System.currentTimeMillis();
        if (readingEnd - readingStart < 60) {
          try {
            Thread.sleep(60 - (readingEnd - readingStart));
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }
  }

  /**
   * get the reading from the left color sensor
   * 
   * @return reading of red light from left sensor
   */
  public static float getLeftSample() {
    return leftSample;
  }

  /**
   * get the reading from the right color sensor
   * 
   * @return reading of red light from right sensor
   */
  public static float getRightSample() {
    return rightSample;
  }

  /**
   * Whether of not the left sensor is detecting a black line
   * 
   * @return boolean value indicating that the left sensor has transitioned onto a black line
   */
  public static boolean leftDetectBlackLine() {
    if (leftDer < 0 && Math.abs(leftDer) > SENSOR_DERIVATIVE_THRESHOLD) {
      navigation.stopTheRobot();
      Sound.beep();
      return true;
    } else {
      return false;
    }
  }

  /**
   * Whether of not the right sensor is detecting a black line
   * 
   * @return boolean value indicating that the right sensor has transitioned onto a black line
   */
  public static boolean rightDetectBlackLine() {
    if (rightDer < 0 && Math.abs(rightDer) > SENSOR_DERIVATIVE_THRESHOLD) {
      navigation.stopTheRobot();
      Sound.twoBeeps();
      return true;
    } else {
      return false;
    }
  }

  /**
   * corrects the odometer according to the nearest black line using internal value readings
   */
  public static void correctXYT() {
    int updatedTheta;
    double[] currXYT = odometer.getXYT();
    /*
     * Resets the angle (since we are only driving in cardinal directions, there are only 4 possibilities
     */
    if (currXYT[2] >= 315 || currXYT[2] < 45) {
      updatedTheta = 0;
      odometer.setTheta(updatedTheta);
      correctY(currXYT, updatedTheta);
    } else if (currXYT[2] >= 45 && currXYT[2] < 135) {
      updatedTheta = 90;
      odometer.setTheta(updatedTheta);
      correctX(currXYT, updatedTheta);
    } else if (currXYT[2] >= 135 && currXYT[2] < 225) {
      updatedTheta = 180;
      odometer.setTheta(updatedTheta);
      correctY(currXYT, updatedTheta);
    } else if (currXYT[2] >= 225 && currXYT[2] < 315) {
      updatedTheta = 270;
      odometer.setTheta(updatedTheta);
      correctX(currXYT, updatedTheta);
    }
  }

  /**
   *
   * Reset the X position based on what tile the robot thinks it is on, if the robot is less than half a tile away from
   * its current position in each direction, correction is possible
   *
   * @param currXYT
   * @param updatedTheta
   */
  private static void correctX(double[] currXYT, double updatedTheta) {
    double currX = currXYT[0];
    currX = TILE_SIZE * (Math.round((currX) / TILE_SIZE));
    if (updatedTheta == 90) {
      odometer.setX(currX + Resources.SENSOR_RADIUS);
    } else {
      odometer.setX(currX - Resources.SENSOR_RADIUS);
    }
  }

  /**
   *
   * Reset the Y position based on what tile the robot thinks it is on, if the robot is less than half a tile away from
   * its current position in each direction, correction is possible
   *
   * @param currXYT
   * @param updatedTheta
   */
  private static void correctY(double[] currXYT, double updatedTheta) {
    double currY = currXYT[1];
    currY = TILE_SIZE * (Math.round((currY) / TILE_SIZE));
    if (updatedTheta == 0) {
      odometer.setX(currY + Resources.SENSOR_RADIUS);
    } else {
      odometer.setX(currY - Resources.SENSOR_RADIUS);
    }
  }

  public static void resetLineDetection() {
    isCorrecting = false;
    leftLineDetected = false;
    rightLineDetected = false;

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
    }
  }

  public void sleep() {
    wait = true;
  }

  public void wake() {
    wait = false;
  }

  public boolean[] getLineDetectionStatus() {

    boolean[] result = {leftLineDetected, rightLineDetected};
    return result;
  }

}
