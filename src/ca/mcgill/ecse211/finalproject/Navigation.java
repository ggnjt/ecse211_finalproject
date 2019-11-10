package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.ACCELERATION;
import static ca.mcgill.ecse211.finalproject.Resources.ARENA_X;
import static ca.mcgill.ecse211.finalproject.Resources.ARENA_Y;
import static ca.mcgill.ecse211.finalproject.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.TRACK;
import static ca.mcgill.ecse211.finalproject.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.finalproject.Resources.colorPoller;
import static ca.mcgill.ecse211.finalproject.Resources.leftMotor;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.pathFinder;
import static ca.mcgill.ecse211.finalproject.Resources.rightMotor;
import ca.mcgill.ecse211.finalproject.phase2.ColorPoller;
import ca.mcgill.ecse211.finalproject.phase2.PathFinder;

/**
 * Class where all of the navigation of the robot is handled
 * 
 * @author yp
 * @author elias
 */
public class Navigation {
  /**
   * orientation of the robot
   * 
   * @author yp
   *
   */
  public enum Orientation {
    NORTH, SOUTH, EAST, WEST
  }

  /**
   * traveling state of the robot, either traveling, correcting angle or detected an obstacle
   * 
   * @author yp
   *
   */
  public enum TravelingMode {
    TRAVELING, CORRECTING, OBSTACLE_ENCOUNTERED
  }

  /**
   * set Speed
   */
  public void setSpeed(int speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
  }

  /**
   * current position and orientation of the robot on the grid
   */
  public Orientation orientation = Orientation.NORTH; // initial orientation

  /**
   * current state of the robot
   */
  public TravelingMode navigationMode = TravelingMode.TRAVELING;

  /**
   * current x tile coordinate of the robot
   */
  public int xTile = 0;

  /**
   * current x tile coordinate of the robot
   */
  public int yTile = 0;

  /**
   * current target x coordinate
   */
  public int targetX = 0;

  /**
   * current target y coordinate
   */
  public int targetY = 0;

  /**
   * flag for successful move
   */
  public volatile boolean moveSuccessful = false;

  /**
   * flag for interruption
   */
  public volatile boolean interrupted = false;

  /**
   * a size 2 array representing the previous move made by the robot
   */
  public int[] currentMove = {0, 0};

  /**
   * Constructor for the Navigation class.
   */
  public Navigation() {
    setSpeed(FORWARD_SPEED);
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance
   * @return the wheel rotations necessary to cover the distance
   */
  public int convertDistance(double distance) {
    return (int) ((180 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public int convertAngle(double angle) {
    return convertDistance((Math.PI * TRACK * angle) / 360.0);
  }

  /**
   * move forward until the robot encounters an obstacle or finishes the movement of one tile length
   * 
   * @return boolean which returns true if the robot finishes without encountering an obstacle
   */

  public void backUp() {
    System.out.println("backing up...");
    double distance = 0;
    switch (orientation) {
      case NORTH:
        distance = odometer.getXYT()[1] - ((double) yTile + 0.5) * TILE_SIZE;
        break;
      case EAST:
        distance = odometer.getXYT()[0] - ((double) xTile + 0.5) * TILE_SIZE;
        break;
      case SOUTH:
        distance = -(odometer.getXYT()[1] - ((double) yTile + 0.5) * TILE_SIZE);
        break;
      case WEST:
        distance = -(odometer.getXYT()[0] - ((double) xTile + 0.5) * TILE_SIZE);
        break;
    }
    leftMotor.rotate(-convertDistance(distance), true);
    rightMotor.rotate(-convertDistance(distance), false);

  }

  /**
   * returns the nearest available square to shoot from which is N square away form the target
   * 
   * @param targetX x coordinates of the target square, starting from 0
   * @param targetY y coordinates of the target square, starting from 0
   * @return the target square coordinates and the angle needed in an int array of size [3]
   */
  public int[] findTarget(int targetX, int targetY) {
    int[] result = new int[3];
    double shortest_dist = 100;
    int[][] notableSquares = {{0, 5}, {4, 4}, {5, 0}, {4, -4}, {0, -5}, {-4, -4}, {-5, 0}, {-4, 4}};
    int[] thetaOptions = {180, 225, 270, 315, 0, 45, 90, 135};
    for (int i = 0; i < notableSquares.length; i++) {
      int[] pair = notableSquares[i];
      boolean ooX = pair[0] + targetX > ARENA_X || pair[0] + targetX < 0;
      boolean ooY = pair[1] + targetY > ARENA_Y || pair[1] + targetY < 0;
      if (ooX || ooY) {
        continue;
      } else {
        double dist = Math.sqrt((pair[0] + targetX) * (pair[0] + targetX) + (pair[1] + targetY) * (pair[1] + targetY));
        if (dist < shortest_dist) {
          result[0] = pair[0] + targetX;
          result[1] = pair[1] + targetY;
          shortest_dist = dist;
          result[2] = (thetaOptions[i] - 90 + 360) % 360;
        }
      }
    }
    return result;
  }

  /**
   * stops the robot in place
   */
  public void stopTheRobot() {
    // isNavigating = false;
    leftMotor.stop(true);
    rightMotor.stop(false);
  }

  /**
   * process a move which takes the form of a size 2 array. The robot will either turn right, turn left or go straight,
   * then move by one tile
   * 
   * @param move a size 2 array representing the move. This can only take the following forms: {1,0}{-1,0}{0,1}{0,-1}
   */
  public void processNextMove(int[] move) {
    targetX = move[0];
    targetY = move[1];
    currentMove = move;
    switch (navigationMode) {
      case TRAVELING:
        System.out.println("Moving!");
        goTo(targetX, targetY);
        break;
      case CORRECTING:
        System.out.println("Correcting!");
        boolean[] lineCorrectionStatus;
        lineCorrectionStatus = colorPoller.getLineDetectionStatus();
        if (lineCorrectionStatus[0] && lineCorrectionStatus[1]) {
          navigationMode = TravelingMode.TRAVELING; // state switch
          setSpeed(FORWARD_SPEED); 
          ColorPoller.correctXYT();
          ColorPoller.resetLineDetection();
          goTo(targetX, targetY);
        } else if (ColorPoller.leftLineDetected) {
          rightMotor.forward();
        } else if (ColorPoller.rightLineDetected) {
          leftMotor.forward();
        }
        break;
      case OBSTACLE_ENCOUNTERED:
        PathFinder.resetMap();
        pathFinder.setObstacle(xTile, yTile);
        break;
    }
    if (moveSuccessful) {
      xTile = move[0];
      yTile = move[1];
    }
  }

  // ============lab 3 stuff==============//

  // This is a blocking GoTo (blocks other threads)
  private void goTo(int X, int Y) {
    moveSuccessful = false;
    interrupted = false;
    double currentX = odometer.getXYT()[0];
    double currentY = odometer.getXYT()[1];
    double currentTheta = odometer.getXYT()[2];
    double Xtarget = ((double) X + 0.5) * TILE_SIZE;
    double Ytarget = ((double) Y + 0.5) * TILE_SIZE;

    double X2go = Xtarget - currentX;
    double Y2go = Ytarget - currentY;

    double angleTarget = Math.atan(X2go / Y2go) / Math.PI * 180;
    if (Y2go < 0) {
      angleTarget += 180;
    }
    double angleDeviation = angleTarget - currentTheta;

    if (angleDeviation > 180) {
      angleDeviation -= 360;
    } else if (angleDeviation < -180) {
      angleDeviation += 360;
    }

    
    colorPoller.sleep();
    leftMotor.rotate(convertAngle(angleDeviation), true);
    rightMotor.rotate(-convertAngle(angleDeviation), false);
    colorPoller.wake();

    double distance2go = Math.sqrt(X2go * X2go + Y2go * Y2go);

    leftMotor.rotate(convertDistance(distance2go), true);
    rightMotor.rotate(convertDistance(distance2go), false);

    synchronized (this) {
      if (!interrupted) {
        moveSuccessful = true;
      }
    }
  }
}
