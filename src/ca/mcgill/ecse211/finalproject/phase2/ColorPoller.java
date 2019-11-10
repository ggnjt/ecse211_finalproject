package ca.mcgill.ecse211.finalproject.phase2;

import static ca.mcgill.ecse211.finalproject.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.leftColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.leftMotor;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.rightColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.rightMotor;
import lejos.ev3.*;
import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
import ca.mcgill.ecse211.finalproject.Resources;

/**
 * Class which takes care of color sensor polling, signal filtering as well as odometry correction using the color
 * sensors
 * 
 * @author yp
 *
 */
public class ColorPoller implements Runnable {


  /**
   * whether or not the left sensor is currently detecting a black line
   */
  private boolean leftLineDetected = false;
  /**
   * whether or not the right sensor is currently detecting a black line
   */
  private boolean rightLineDetected = false;

  /**
   * make the thread wait while turning
   */
  public static boolean wait = false;

  /**
   * leftSampler
   */
  ColorSampler leftSampler;

  /**
   * Right Sampler
   */
  ColorSampler rightSampler;

  /**
   * leftColorThread
   */
  Thread leftColorThread;

  /**
   * RightColorThread
   */
  Thread rightColorThread;

  public ColorPoller() {
    super();
    leftSampler = new ColorSampler(leftColorSensor);
    rightSampler = new ColorSampler(rightColorSensor);

    leftColorThread = new Thread(leftSampler);
    rightColorThread = new Thread(rightSampler);

    leftColorThread.start();
    rightColorThread.start();
  }

  /**
   * run method for the two color poller
   */
  @Override
  public void run() {
    while (true) {
      if (wait) {
        try {
          Thread.sleep(30);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      } else {
        long readingStart, readingEnd;
        readingStart = System.currentTimeMillis();

        switch (navigation.navigationMode) {
          case TRAVELING:
        	  leftLineDetected = leftSampler.getBlackLine();
              rightLineDetected = rightSampler.getBlackLine();
              if (leftLineDetected || rightLineDetected) {
                navigation.navigationMode = TravelingMode.CORRECTING;
                navigation.setSpeed(40);
                navigation.moveSuccessful = false;
                navigation.stopTheRobot();
              }
              break;
          case OBSTACLE_ENCOUNTERED:
          case CORRECTING:
            leftLineDetected = leftLineDetected || leftSampler.getBlackLine();
            rightLineDetected = rightLineDetected || rightSampler.getBlackLine();

            if (leftLineDetected && rightLineDetected) {
              // clear the line
              leftMotor.rotate(4, true);
              rightMotor.rotate(4, false);

              // Correct
              correctXYT();

              navigation.setSpeed(FORWARD_SPEED);
              navigation.navigationMode = TravelingMode.TRAVELING;
              resetLineDetection();
            } else if (leftLineDetected) {
              rightMotor.forward();
            } else if (rightLineDetected) {
              leftMotor.forward();
            }
            break;
        }

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
   * corrects the odometer according to the nearest black line using internal value readings
   */
  public static void correctXYT() {
    int updatedTheta = 0;
    double[] currXYT = odometer.getXYT();
   
    /*
     * Resets the angle (since we are only driving in cardinal directions, there are only 4 possibilities
     */
    
    if (currXYT[2] >= 45 && currXYT[2] < 135) {
      updatedTheta = 90;
    } else if (currXYT[2] >= 135 && currXYT[2] < 225) {
      updatedTheta = 180;
    } else if (currXYT[2] >= 225 && currXYT[2] < 315) {
      updatedTheta = 270;
    } else {
      updatedTheta = 0;
    }

    odometer.setTheta(updatedTheta);
    correctX(currXYT[0], updatedTheta);
  }

  /**
   *
   * Reset the X position based on what tile the robot thinks it is on, if the robot is less than half a tile away from
   * its current position in each direction, correction is possible
   *
   * @param currXYT
   * @param updatedTheta
   */
  private static void correctX(double currX, double updatedTheta) {
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

  public void resetLineDetection() {
    leftLineDetected = false;
    rightLineDetected = false;

    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
    }
  }

  public void sleep() {
    wait = true;
  }

  public void wake() {
    wait = false;
  }
}
