package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.ACCELERATION;
import static ca.mcgill.ecse211.finalproject.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.finalproject.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.TRACK;
import static ca.mcgill.ecse211.finalproject.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.finalproject.Resources.WPOINT_RAD;
import static ca.mcgill.ecse211.finalproject.Resources.leftMotor;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.rightMotor;

import java.util.Arrays;

import lejos.hardware.Sound;

/**
 * Class where all of the navigation of the robot is handled
 * 
 * @author yp
 *
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
	 * traveling state of the robot, either traveling, correcting angle or detected
	 * an obstacle
	 * 
	 * @author yp
	 *
	 */
	public enum TravelingMode {
		TRAVELING, CORRECTING, OBSTACLE_ENCOUNTERED
	}

	private static boolean navigationRunning = false;
	/**
	 * current position and orientation of the robot on the grid
	 */
	public static Orientation orientation = Orientation.NORTH; // initial orientation
	/**
	 * current state of the robot
	 */
	public static TravelingMode navigationMode = TravelingMode.TRAVELING;

	/**
	 * current x tile coordinate of the robot
	 */
	public static int xTile = 0;
	/**
	 * current x tile coordinate of the robot
	 */
	public static int yTile = 8;

	/**
	 * a size 2 array representing the previous move made by the robot
	 */
	private static int[] previousMove = { 0, 0 };

	/**
	 * Constructor for the Navigation class.
	 */
	public Navigation() {
//    isNavigating = false;
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.setAcceleration(ACCELERATION);
	}

//  /**
//   * start navigating
//   */
//  public static void startNavigating() {
//    isNavigating = true;
//  }

//  /**
//   * The main method used to travel to a waypoint. The method will loop at approximately 20 Hz and make sure the robot
//   * is on the correct path towards the waypoint. It will call turnTo() if the robot needs to make a change in heading.
//   * It will also call avoidObject() if the robot is about to run into an obstacle.
//   * 
//   * @param x the X coordinate of the waypoint
//   * @param y the Y coordinate of the waypoint
//   */
//  public static void travelTo(double x, double y) { //We could scrape this in the final version of the code because it is not used
//
//    position = odometer.getXYT();
//    vectorX = x - position[0];
//    vectorY = y - position[1];
//
//    while (distance(vectorX, vectorY) > WPOINT_RAD) {
//      position = odometer.getXYT(); // Get position of the robot from the odometer
//      // Update the vectors from the current position to the waypoint
//      vectorX = x - position[0];
//      vectorY = y - position[1];
//      // Update the heading, and ensure it stays between 0 and 360 degrees
//      heading = Math.toDegrees(Math.atan2(vectorX, vectorY));
//      heading = (heading + 360) % 360;
//      Resources.LCD.drawString("Heading: " + Double.toString(heading), 0, 3);
//      // If the robot isn't too close to the waypoint, allow it to correct its heading
//      // by rotating
//      if (distance(vectorX, vectorY) > (2 * WPOINT_RAD)) {
//        turnTo(heading);
//      }
//
//      leftMotor.setSpeed(FORWARD_SPEED);
//      rightMotor.setSpeed(FORWARD_SPEED);
//      leftMotor.forward();
//      rightMotor.forward();
//      try {
//        Thread.sleep(50);
//      } catch (Exception e) {
//      }
//
//    }
//    leftMotor.stop(true);
//    rightMotor.stop(false);
//    Sound.twoBeeps(); // Beep when it has reached a waypoint
//  }

	/**
	 * Rotates the robot to an absolute angle theta. It also ensures the robot turns
	 * the minimal angle to get to theta.
	 * 
	 * @param theta the absolute angle the robot should turn to in degrees
	 */
	public static void turnTo(double theta) {
		double angleDiff = theta - odometer.getXYT()[2];
		// Don't correct the angle if it is within a certain threshold
		if (Math.abs(angleDiff) < 3.0 || Math.abs(angleDiff) > 357.0) {
			return;
		}
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		// This ensures the robot uses the minimal angle when turning to theta
		if (Math.abs(angleDiff) > 180.0) {
			angleDiff = Math.signum(angleDiff) * 360.0 - angleDiff;
			leftMotor.rotate(convertAngle(-angleDiff), true);
			rightMotor.rotate(convertAngle(angleDiff), false);
		} else {
			leftMotor.rotate(convertAngle(angleDiff), true);
			rightMotor.rotate(convertAngle(-angleDiff), false);
		}
	}

//  /**
//   * Returns a boolean of whether or not the robot is currently navigating to a waypoint.
//   * 
//   * @return true if the robot is currently navigating to a waypoint.
//   */
//  public static boolean getNavigating() {
//    return isNavigating;
//  }

	/**
	 * Converts input distance to the total rotation of each wheel needed to cover
	 * that distance.
	 * 
	 * @param distance
	 * @return the wheel rotations necessary to cover the distance
	 */
	public static int convertDistance(double distance) {
		return (int) ((180 * distance) / (Math.PI * WHEEL_RAD));
	}

	/**
	 * Converts input angle to the total rotation of each wheel needed to rotate the
	 * robot by that angle.
	 * 
	 * @param angle
	 * @return the wheel rotations necessary to rotate the robot by the angle
	 */
	public static int convertAngle(double angle) {
		return convertDistance((Math.PI * TRACK * angle) / 360.0);
	}

	/**
	 * Calculates the euclidian distance given an X and Y distance, in cm.
	 * 
	 * @param deltaX X distance
	 * @param deltaY Y distance
	 * @return Euclidean distance
	 */
	private static double distance(double deltaX, double deltaY) {
		return Math.sqrt((Math.pow((deltaX), 2) + Math.pow((deltaY), 2)));
	}

	/**
	 * moves the robot forward by x many tile lengths this is a blocking method
	 * 
	 * @param i number of tile lengths
	 */
	public static void moveForwardByTile(double i) {
		leftMotor.rotate(convertDistance(TILE_SIZE * i), true);
		rightMotor.rotate(convertDistance(TILE_SIZE * i), false);
	}

	/**
	 * move forward until the robot encounters an obstacle or finishes the movement
	 * of one tile length
	 * 
	 * @return boolean which returns true if the robot finishes without encountering
	 *         an obstacle
	 */
	public static boolean moveForwardOneTile() {
		navigationMode = TravelingMode.TRAVELING;
		navigationRunning = true;
		final Thread moveOneTile = new Thread() {
			public void run() {
				whileloop: while (true) {
					switch (navigationMode) {
					case TRAVELING:
						leftMotor.forward();
						rightMotor.forward();
						if (UltrasonicObstacleDetector.obstacleDetected) {
							navigationMode = TravelingMode.OBSTACLE_ENCOUNTERED;
							break whileloop;
						}
						if (ColorPoller.isCorrecting) {
							navigationMode = TravelingMode.CORRECTING;
						}
						double[] currentXYT = odometer.getXYT();
						switch (orientation) {
						case NORTH:
							if (currentXYT[1] >= yTile * Resources.TILE_SIZE + 0.5 * TILE_SIZE) {
								break whileloop;
							}
							break;
						case SOUTH:
							if (currentXYT[1] >= yTile * Resources.TILE_SIZE - 0.5 * TILE_SIZE) {
								break whileloop;
							}
							break;
						case EAST:
							if (currentXYT[0] >= xTile * Resources.TILE_SIZE + 0.5 * TILE_SIZE) {
								break whileloop;
							}
							break;
						case WEST:
							if (currentXYT[0] >= xTile * Resources.TILE_SIZE - 0.5 * TILE_SIZE) {
								break whileloop;
							}
							break;
						}
						break;
					case CORRECTING:
						boolean[] lineCorrectionStatus;
						lineCorrectionStatus = Resources.colorPoller.getLineDetectionStatus();
						if (lineCorrectionStatus[0] && !lineCorrectionStatus[1]) {
							leftMotor.stop();
						} else if (lineCorrectionStatus[0] && lineCorrectionStatus[1]) {
							rightMotor.stop();
						} else if (lineCorrectionStatus[0] && lineCorrectionStatus[1]) {
							Resources.colorPoller.resetLineDetection();
							leftMotor.forward();
							rightMotor.forward();
							navigationMode = TravelingMode.TRAVELING;
						}
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
				navigationRunning = false;
			}
		};
		moveOneTile.start();

		while (navigationRunning) {
			// not sure if this is okay...
			try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		try {
			moveOneTile.join(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		if (navigationMode == TravelingMode.OBSTACLE_ENCOUNTERED) {
			return false;
		} else if (navigationMode == TravelingMode.TRAVELING) {
			return true;
		} else {
			System.out.println("somehting is wrong, the robot finished while correcting...");
			return false;
		}
	}

	public static void backUp() {
		double distance = 0;
		switch (orientation) {
		case NORTH:
			distance = odometer.getXYT()[1] - ((double) yTile + 0.5) * Resources.TILE_SIZE;
			break;
		case EAST:
			distance = odometer.getXYT()[0] - ((double) xTile + 0.5) * Resources.TILE_SIZE;
			break;
		case SOUTH:
			distance = -(odometer.getXYT()[1] - ((double) yTile + 0.5) * Resources.TILE_SIZE);
			break;
		case WEST:
			distance = -(odometer.getXYT()[0] - ((double) xTile + 0.5) * Resources.TILE_SIZE);
			break;
		}
		leftMotor.rotate(-convertDistance(distance), true);
		rightMotor.rotate(-convertDistance(distance), false);

	}

	/**
	 * turns the robot 90 degrees left and sleep the color sensor threads
	 */
	public static void turnLeft() {
		Resources.colorPoller.sleep();
		leftMotor.rotate(convertAngle(-90.0), true);
		rightMotor.rotate(convertAngle(90.0), false);
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
	public static void turnRight() {
		Resources.colorPoller.sleep();
		leftMotor.rotate(convertAngle(90.0), true);
		rightMotor.rotate(convertAngle(-90.0), false);
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
	 * returns the nearest available square to shoot from which is N square away
	 * form the target
	 * 
	 * @param targetX x coordinates of the target square, starting from 0
	 * @param targetY y coordinates of the target square, starting from 0
	 * @return the target square coordinates and the angle needed in an int array of
	 *         size [3]
	 */
	public static int[] findTarget(int targetX, int targetY) {
		int[] result = new int[3];
		double shortest_dist = 100;
		int[][] notableSquares = { { 0, 5 }, { 4, 4 }, { 5, 0 }, { 4, -4 }, { 0, -5 }, { -4, -4 }, { -5, 0 },
				{ -4, 4 } };
		int[] thetaOptions = { 180, 225, 270, 315, 0, 45, 90, 135 };

		for (int i = 0; i < notableSquares.length; i++) {
			int[] pair = notableSquares[i];
			boolean ooX = pair[0] + targetX > Resources.ARENA_X || pair[0] + targetX < 0;
			boolean ooY = pair[1] + targetY > Resources.ARENA_Y || pair[1] + targetY < 0;
			if (ooX || ooY) {
				continue;
			} else {
				double dist = Math
						.sqrt((pair[0] + targetX) * (pair[0] + targetX) + (pair[1] + targetY) * (pair[1] + targetY));
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
	 * moves the robot to the optimal square and take aim
	 * 
	 * @param targetX target square X coordinate
	 * @param targetY target square Y coordinate
	 */
	public static void getReadyToShoot(int targetX, int targetY) {
		int[] destination = findTarget(targetX, targetY);
		moveForwardByTile(destination[1]);
		turnRight();
		moveForwardByTile(destination[0]);
		turnTo(destination[2]);
		if (destination[2] % 90 > 0) {
			moveForwardByTile(0.5); // Minor correction for corner cases
		}
	}

	/**
	 * stops the robot in place
	 */
	public static void stopTheRobot() {
//    isNavigating = false;
		Resources.leftMotor.stop(true);
		Resources.rightMotor.stop(false);
	}

	/**
	 * process a move which takes the form of a size 2 array. The robot will either
	 * turn right, turn left or go straight, then move by one tile
	 * 
	 * @param move a size 2 array representing the move. This can only take the
	 *             following forms: {1,0}{-1,0}{0,1}{0,-1}
	 */
	public static void processNextMove(int[] move) {
		// check if there is an obstacle
		boolean success = true;
		if (previousMove[0] == 0 && previousMove[1] == 0) {
			// TODO: initialization
			boolean verti = move[0] == 0;
			// if (verti) {
			// turnTo(move[1] == 1? 90:-90);
			// }
			// else {
			// turnTo(move[0] == 1? 0:180);
			// }
			moveForwardOneTile();
		} else if (Arrays.equals(move, previousMove)) {
			if (!moveForwardOneTile()) {
				backUp();
				success = false;
			}
		} else {
			if (previousMove[0] == move[1] && previousMove[1] == -move[0]) {
				turnLeft();
				if (!moveForwardOneTile()) {
					backUp();
					success = false;
				}
			} else if (previousMove[0] == -move[1] && previousMove[1] == move[0]) {
				turnRight();
				if (!moveForwardOneTile()) {
					backUp();
					success = false;
				}
			} else if (previousMove[0] == -move[0] && previousMove[1] == -move[1]) {
				turnRight();
				turnRight();
				if (!moveForwardOneTile()) {
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
			}
			else {
				previousMove = move;
			}
		}
	}
}
