package ca.mcgill.ecse211.finalproject;

import java.util.Arrays;
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
    Resources.leftMotor.setSpeed(speed);
    Resources.rightMotor.setSpeed(speed);
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
  public int yTile = 8;

  /**
   * a size 2 array representing the previous move made by the robot
   */
  private int[] previousMove = {0, 0};

  /**
   * Constructor for the Navigation class.
   */
  public Navigation() {
    Resources.leftMotor.setAcceleration(Resources.ACCELERATION);
    Resources.rightMotor.setAcceleration(Resources.ACCELERATION);
  }


  /**
   * Rotates the robot to an absolute angle theta. It also ensures the robot turns the minimal angle to get to theta.
   * 
   * @param theta the absolute angle the robot should turn to in degrees
   */
  public void turnTo(double theta) {
    double angleDiff = theta - Resources.odometer.getXYT()[2];
    // Don't correct the angle if it is within a certain threshold
    if (Math.abs(angleDiff) < 3.0 || Math.abs(angleDiff) > 357.0) {
      return;
    }
    Resources.leftMotor.setSpeed(Resources.ROTATE_SPEED);
    Resources.rightMotor.setSpeed(Resources.ROTATE_SPEED);
    // This ensures the robot uses the minimal angle when turning to theta
    if (Math.abs(angleDiff) > 180.0) {
      angleDiff = Math.signum(angleDiff) * 360.0 - angleDiff;
      Resources.leftMotor.rotate(convertAngle(-angleDiff), true);
      Resources.rightMotor.rotate(convertAngle(angleDiff), false);
    } else {
      Resources.leftMotor.rotate(convertAngle(angleDiff), true);
      Resources.rightMotor.rotate(convertAngle(-angleDiff), false);
    }
  }

  // /**
  // * Returns a boolean of whether or not the robot is currently navigating to a
  // waypoint.
  // *
  // * @return true if the robot is currently navigating to a waypoint.
  // */
  // public boolean getNavigating() {
  // return isNavigating;
  // }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance
   * @return the wheel rotations necessary to cover the distance
   */
  public int convertDistance(double distance) {
    return (int) ((180 * distance) / (Math.PI * Resources.WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public int convertAngle(double angle) {
    return convertDistance((Math.PI * Resources.TRACK * angle) / 360.0);
  }

  /**
   * move forward until the robot encounters an obstacle or finishes the movement of one tile length
   * 
   * @return boolean which returns true if the robot finishes without encountering an obstacle
   */
  public boolean moveForwardByOneTile() {
    navigationMode = TravelingMode.TRAVELING;

    whileloop: while (true) {
      switch (navigationMode) {
        case TRAVELING:
          Resources.leftMotor.forward();
          Resources.rightMotor.forward();
          if (UltrasonicObstacleDetector.obstacleDetected) {
            navigationMode = TravelingMode.OBSTACLE_ENCOUNTERED;
            break whileloop;
          }

          if (ColorPoller.isCorrecting) {
            navigationMode = TravelingMode.CORRECTING;
          }

          double[] currentXYT = Resources.odometer.getXYT();

          switch (orientation) {
            case NORTH:
              if (currentXYT[1] >= yTile * Resources.TILE_SIZE + 0.5 * Resources.TILE_SIZE) {
                break whileloop;
              }
              break;
            case SOUTH:
              if (currentXYT[1] >= yTile * Resources.TILE_SIZE - 0.5 * Resources.TILE_SIZE) {
                break whileloop;
              }
              break;
            case EAST:
              if (currentXYT[0] >= xTile * Resources.TILE_SIZE + 0.5 * Resources.TILE_SIZE) {
                break whileloop;
              }
              break;
            case WEST:
              if (currentXYT[0] >= xTile * Resources.TILE_SIZE - 0.5 * Resources.TILE_SIZE) {
                break whileloop;
              }
              break;
          }
          break;

        case CORRECTING:
          boolean[] lineCorrectionStatus;
          lineCorrectionStatus = Resources.colorPoller.getLineDetectionStatus();
          if (lineCorrectionStatus[0] && !lineCorrectionStatus[1]) {
            Resources.leftMotor.stop();
          } else if (lineCorrectionStatus[0] && lineCorrectionStatus[1]) {
            Resources.rightMotor.stop();
          } else if (lineCorrectionStatus[0] && lineCorrectionStatus[1]) {
            Resources.colorPoller.resetLineDetection();
            Resources.leftMotor.forward();
            Resources.rightMotor.forward();
            navigationMode = TravelingMode.TRAVELING;
          }
          break;

        case OBSTACLE_ENCOUNTERED:
          break whileloop;
      }
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    stopTheRobot();

    if (navigationMode == TravelingMode.OBSTACLE_ENCOUNTERED) {
      return false;
    } else if (navigationMode == TravelingMode.TRAVELING) {
      return true;
    } else {
      System.out.println("somehting is wrong, the robot finished while correcting...");
      return false;
    }
  }

  public void backUp() {
    double distance = 0;
    switch (orientation) {
      case NORTH:
        distance = Resources.odometer.getXYT()[1] - ((double) yTile + 0.5) * Resources.TILE_SIZE;
        break;
      case EAST:
        distance = Resources.odometer.getXYT()[0] - ((double) xTile + 0.5) * Resources.TILE_SIZE;
        break;
      case SOUTH:
        distance = -(Resources.odometer.getXYT()[1] - ((double) yTile + 0.5) * Resources.TILE_SIZE);
        break;
      case WEST:
        distance = -(Resources.odometer.getXYT()[0] - ((double) xTile + 0.5) * Resources.TILE_SIZE);
        break;
    }
    Resources.leftMotor.rotate(-convertDistance(distance), true);
    Resources.rightMotor.rotate(-convertDistance(distance), false);

  }

  /**
   * turns the robot 90 degrees left and sleep the color sensor threads
   */
  public void turnLeft() {
    Resources.colorPoller.sleep();
    Resources.leftMotor.rotate(convertAngle(-90.0), true);
    Resources.rightMotor.rotate(convertAngle(90.0), false);
    switch (orientation) {
      case NORTH:
        orientation = Orientation.WEST;
        break;
      case EAST:
        orientation = Orientation.NORTH;
        break;
      case SOUTH:
        orientation = Orientation.EAST;
        break;
      case WEST:
        orientation = Orientation.SOUTH;
        break;
    }
    Resources.colorPoller.wake();
  }

  /**
   * turns the robot 90 degrees right and sleep the color sensor threads
   */
  public void turnRight() {
    Resources.colorPoller.sleep();
    Resources.leftMotor.rotate(convertAngle(90.0), true);
    Resources.rightMotor.rotate(convertAngle(-90.0), false);
    switch (orientation) {
      case NORTH:
        orientation = Orientation.EAST;
        break;
      case EAST:
        orientation = Orientation.SOUTH;
        break;
      case SOUTH:
        orientation = Orientation.WEST;
        break;
      case WEST:
        orientation = Orientation.NORTH;
        break;
    }
    Resources.colorPoller.wake();
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
      boolean ooX = pair[0] + targetX > Resources.ARENA_X || pair[0] + targetX < 0;
      boolean ooY = pair[1] + targetY > Resources.ARENA_Y || pair[1] + targetY < 0;
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
    Resources.leftMotor.stop(true);
    Resources.rightMotor.stop(false);
  }

  /**
   * process a move which takes the form of a size 2 array. The robot will either turn right, turn left or go straight,
   * then move by one tile
   * 
   * @param move a size 2 array representing the move. This can only take the following forms: {1,0}{-1,0}{0,1}{0,-1}
   */
  public void processNextMove(int[] move) {
    // check if there is an obstacle
    boolean success = true;
    if (previousMove[0] == 0 && previousMove[1] == 0) {
      // TODO: initialization
      // boolean verti = move[0] == 0;
      // if (verti) {
      // turnTo(move[1] == 1? 90:-90);
      // }
      // else {
      // turnTo(move[0] == 1? 0:180);
      // }
      moveForwardByOneTile();
    } else if (Arrays.equals(move, previousMove)) {
      if (!moveForwardByOneTile()) {
        backUp();
        success = false;
      }
    } else {
      if (previousMove[0] == move[1] && previousMove[1] == -move[0]) {
        turnLeft();
        if (!moveForwardByOneTile()) {
          backUp();
          success = false;
        }
      } else if (previousMove[0] == -move[1] && previousMove[1] == move[0]) {
        turnRight();
        if (!moveForwardByOneTile()) {
          backUp();
          success = false;
        }
      } else if (previousMove[0] == -move[0] && previousMove[1] == -move[1]) {
        turnRight();
        turnRight();
        if (!moveForwardByOneTile()) {
          backUp();
          success = false;
        }
      } else {
        System.out.println("something went terribly wrong");
        success = false;
        // This should never happen if the pathfinder works correctly
      }
      if (!success) {
        PathFinder.resetMap();
        Resources.pathFinder.setObstacle(xTile, yTile);
      } else {
        previousMove = move;
      }
    }
  }
}
