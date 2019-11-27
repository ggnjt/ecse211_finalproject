package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.ACCELERATION;
import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.TRACK;
import static ca.mcgill.ecse211.finalproject.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.finalproject.Resources.leftMotor;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.rightMotor;

import java.util.ArrayList;
import java.util.Arrays;

import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
import ca.mcgill.ecse211.finalproject.phase2.ColorPoller;
import ca.mcgill.ecse211.finalproject.phase2.PathFinder;
import lejos.hardware.Sound;

/**
 * Class where all of the navigation of the robot is handled
 * 
 * @author yp
 * @author elias
 */
public class Navigation {

	/**
	 * traveling state of the robot, either traveling, correcting angle or detected
	 * an obstacle
	 * 
	 * @author yp
	 *
	 */
	public static enum TravelingMode {
		TRAVELING, CORRECTING, OBSTACLE_ENCOUNTERED
	}

	/**
	 * current moving state of the robot
	 */
	public static TravelingMode navigationMode = TravelingMode.TRAVELING;

	/**
	 * current x tile coordinate of the robot
	 */
	public static int xTile = 0;

	/**
	 * current x tile coordinate of the robot
	 */
	public static int yTile = 0;

	/**
	 * target x coordinate of the current move being processed
	 */
	public static int targetX = 0;

	/**
	 * target y coordinate of the current move being processed
	 */
	public static int targetY = 0;

	/**
	 * whether the current move being processed in sucessful
	 */
	public static boolean moveSuccessful = false;

	/**
	 * whether a move has been interrupted by a thread
	 */
	public static boolean interrupted = false;

	/**
	 * Constructor for the Navigation class. sets speed and acceleration of motors
	 */
	public Navigation() {
		setSpeed(Resources.LOW_FORWARD_SPEED);
		leftMotor.setAcceleration(ACCELERATION);
		rightMotor.setAcceleration(ACCELERATION);
	}

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
	 * set for both wheels
	 */
	public static void setSpeed(int speed) {
		synchronized (Resources.leftMotor) {
			synchronized (Resources.rightMotor) {
				leftMotor.setSpeed(speed);
				rightMotor.setSpeed(speed);
			}
		}
	}

	/**
	 * stops the robot in place
	 */
	public static void stopTheRobot() {
		synchronized (Resources.leftMotor) {
			synchronized (Resources.rightMotor) {
				leftMotor.stop(true);
				rightMotor.stop(false);
			}
		}
	}

	/**
	 * process a move which takes the form of a size 2 array. The robot will turn
	 * and move towards the center of that tile
	 * 
	 * @param move a size 2 representing the coordinates of the target tile
	 */
	public static void processNextMove(int[] move) {
		targetX = move[0];
		targetY = move[1];
		switch (navigationMode) {
		case OBSTACLE_ENCOUNTERED:
			Main.sleepFor(70);
			break;
		case TRAVELING:
			goTo(targetX, targetY);
			break;
		case CORRECTING:
			Main.sleepFor(70); // maybe?
			break;

		}
	}

	/**
	 * turn and move to the center of a target tile
	 * 
	 * @param X x-coordinates of the target tile
	 * @param Y y-coordinates of the target tile
	 */
	public static void goTo(int X, int Y) {
		interrupted = false; // fail safe
		// calculation for distance & angle
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
		double distance2go = Math.sqrt(X2go * X2go + Y2go * Y2go);

		// store current speed to reset after
		int speed = Resources.leftMotor.getSpeed();
		// turn off pollers to prevent false reading
		UltrasonicPoller.sleep();
		ColorPoller.sleep();
		// turn to next tile
		synchronized (Resources.leftMotor) {
			synchronized (Resources.rightMotor) {
				setSpeed(Resources.ROTATE_SPEED);
				leftMotor.rotate(convertAngle(angleDeviation), true);
				rightMotor.rotate(-convertAngle(angleDeviation), false);
				setSpeed(speed);
			}
		}
		// wake the pollers
		ColorPoller.wake();
		UltrasonicPoller.wake();

		// if the vehicle turned, the US reading will be affected and need to be reset
		// and needs more time to get a new reading. If the vehicle was moving straight,
		// then it would have detected the obstacle while moving forward so no need for
		// reset
		// SLEEP TO MAKE SURE THAT THE POLLER READING IS ACCURATE
		if (Math.abs(angleDeviation) > 60) {
			UltrasonicPoller.resetDetection();
			Main.sleepFor(1000);
		} else {
			Main.sleepFor(400);
		}
		// obstacle detectingafter turning, so move was interrupted
		if (!PathFinder.isFacingAWall() && UltrasonicPoller.hasDetected()) {
			moveSuccessful = false;
			interrupted = true;
			synchronized (navigationMode) {
				navigationMode = TravelingMode.OBSTACLE_ENCOUNTERED;
				return;
			}
		}

		// move the vehicle forward
		// no synchronized block because the poller should be able to interrupt this
		// movement upon detection of black line
		leftMotor.rotate(convertDistance(distance2go), true);
		rightMotor.rotate(convertDistance(distance2go), false);

		// update the tile if movement successfull
		if (!interrupted) {
			moveSuccessful = true;
		}
		if (moveSuccessful) {
			xTile = (int) (odometer.getXYT()[0] / TILE_SIZE);
			yTile = (int) (odometer.getXYT()[1] / TILE_SIZE);
		}
		// obstacle detection after successfully reaching the center of tile, mark as
		// success and let the navigationMode catch the need for new path
		if (!PathFinder.isFacingAWall() && UltrasonicPoller.hasDetected()) {
			synchronized (navigationMode) {
				navigationMode = TravelingMode.OBSTACLE_ENCOUNTERED;
			}
		}
		// prevet threading issue
		Main.sleepFor(100);
	}

	/**
	 * turn to the target absolute theta
	 * 
	 * @param theta target angle to turn towards, 0 being facing North
	 */
	public static void turnTo(double theta) {
		theta = theta % 360;
		double angleDiff = theta - odometer.getXYT()[2];
		synchronized (leftMotor) {
			synchronized (rightMotor) {
				leftMotor.setSpeed(Resources.ROTATE_SPEED);
				rightMotor.setSpeed(Resources.ROTATE_SPEED);
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
		}

	}

	/**
	 * moves the robot to the lower left corner of a square
	 */
	public static void goToLowerLeftCorner() {
		ColorPoller.sleep();
		turnTo(225);
		synchronized (leftMotor) {
			synchronized (rightMotor) {
				Resources.leftMotor.rotate(convertDistance(0.73 * TILE_SIZE), true);
				Resources.rightMotor.rotate(convertDistance(0.73 * TILE_SIZE), false);
			}
		}
	}

	/**
	 * recenters the robot from the lower left corner
	 */
	public static void reCenter() {
		turnTo(45);
		synchronized (leftMotor) {
			synchronized (rightMotor) {
				Resources.leftMotor.rotate(convertDistance(0.73 * TILE_SIZE), true);
				Resources.rightMotor.rotate(convertDistance(0.73 * TILE_SIZE), false);
			}
		}
		odometer.setTheta(45);
		xTile = (int) (odometer.getXYT()[0] / TILE_SIZE);
		yTile = (int) (odometer.getXYT()[1] / TILE_SIZE);
		PathFinder.resetMap();
		ColorPoller.wake();
	}

	/**
	 * makes the robot run along the list of moves
	 * 
	 * @param moves List of moves in the format of square coordinates
	 * @return the list of moves has been successfully achieves in its entirety
	 */
	public static boolean run(ArrayList<int[]> moves) {
		for (int[] move : moves) {
			System.out.println(Arrays.toString(move)); // prints the tile it is moving to
			UltrasonicPoller.resetDetection();
			setSpeed(Resources.LOW_FORWARD_SPEED);
			// since a move ends at the center, we are moving towards a line now so lower
			// the speed
			processNextMove(move);
			while (!moveSuccessful || interrupted) { // move was interrupted by a line / obstacle
				UltrasonicPoller.resetDetection();
				if (navigationMode == TravelingMode.TRAVELING) { // after the black line TODO: if mid-line is annoying
//					if (targetY == 4) { // avoid the annoying black line in the middle of the arena
//						ColorPoller.sleep(); // this doesn't work too well
//					}
					processNextMove(move); // keeps on moving to the center
				} else if (navigationMode == TravelingMode.OBSTACLE_ENCOUNTERED) {
					UltrasonicPoller.resetDetection();
					break;
				} else {
					Main.sleepFor(70); // let colorPoller take over
				}
//				ColorPoller.wake(); // wake it just in case it finished sleeping for the middle seam TODO: if mid-line is annoying
			}
			if (navigationMode == TravelingMode.OBSTACLE_ENCOUNTERED) {
				// this catches the case where the list of moves is not complete. Path needs
				// reset
				if (PathFinder.setObstacle()) { // if obstacle is in the arena and not a wall
					PathFinder.resetMap();
					Main.moves = PathFinder.findPath();
					navigationMode = TravelingMode.TRAVELING;
					moveSuccessful = false;
					Navigation.interrupted = true;
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * moves the robot by a set distance
	 * 
	 * @param distance distance to move by in cm
	 */
	public static void moveByDistance(double distance) {
		synchronized (leftMotor) {
			synchronized (rightMotor) {
				Resources.leftMotor.rotate(convertDistance(distance), true);
				Resources.rightMotor.rotate(convertDistance(distance), false);
			}
		}
	}

	/**
	 * goes to a corner, aims, shoots, resets target to home and re-centers to
	 * middle of square
	 */
	public static void launchManeuver() {
		goToLowerLeftCorner();
		turnTo(PathFinder.launchAngle + 180);
		moveByDistance((-PathFinder.launchAdjustment) * Resources.TILE_SIZE - 0.7 * Resources.TILE_SIZE);
		Sound.beep();
		Main.sleepFor(100);
		Sound.beep();
		Main.sleepFor(100);
		Sound.beep();
		Main.sleepFor(100);
		Resources.shooterMotor.rotate(180);
		Main.sleepFor(1000);
		Resources.shooterMotor.flt(true);
		// Resources.pathFinder.printMap();
		PathFinder.letsGoHome();
		moveByDistance((PathFinder.launchAdjustment) * Resources.TILE_SIZE + 0.7 * Resources.TILE_SIZE);
		reCenter();
		Main.moves = PathFinder.findPath();
	}

//	public static void localizeWithinSquare() {
//		ColorPoller.sleep();
//		allignInFront();
//		setSpeed(180);
//		synchronized (leftMotor) {
//			synchronized (rightMotor) {
//				Resources.leftMotor.rotate(convertDistance(-TILE_SIZE/2d+4.5), true);
//				Resources.rightMotor.rotate(convertDistance(-TILE_SIZE/2d+4.5), false);
//			}
//		}
//		Main.sleepFor(50);
//		synchronized (leftMotor) {
//			synchronized (rightMotor) {
//				Resources.leftMotor.rotate(convertAngle(90), true);
//				Resources.rightMotor.rotate(convertAngle(-90), false);
//			}
//		}
//		allignInFront();
//		synchronized (leftMotor) {
//			synchronized (rightMotor) {
//				Resources.leftMotor.rotate(convertDistance(-TILE_SIZE/2d+4.5), true);
//				Resources.rightMotor.rotate(convertDistance(-TILE_SIZE/2d+4.5), false);
//			}
//		}
//		double currTheta = odometer.getXYT()[2];
//		if (currTheta >= 45 && currTheta < 135) {
//			currTheta = 90d; // facing EAST
//		} else if (currTheta >= 135 && currTheta < 225) {
//			currTheta = 180d; // facing SOUTH
//		} else if (currTheta >= 225 && currTheta < 315) {
//			currTheta = 270d; // facing WEST
//		} else {
//			currTheta = 0d; // facing NORTH
//		}
//		Resources.odometer.setTheta(currTheta);
//		ColorPoller.wake();
//		return;
//	}
//	
//	private static void allignInFront() {
//		boolean leftLineDetected, rightLineDetected;
//		int leftCounter = 0, rightCounter = 0;
//		boolean statechange = false;
//		setSpeed(80);
//		synchronized (leftMotor) {
//			synchronized (rightMotor) {
//				leftMotor.forward();
//				rightMotor.forward();
//			}
//		}
//		while (true) {
//			leftLineDetected = ColorPoller.leftSampler.getBlackLine();
//			rightLineDetected = ColorPoller.rightSampler.getBlackLine();
//			if (!statechange) {
//				if (leftLineDetected || rightLineDetected) {
//					// line detection during traveling, state change and slow the robot down
//					Navigation.stopTheRobot();
//					statechange = true;
//				} else {
//					continue;
//				}
//			} else {
//				leftLineDetected = leftLineDetected || ColorPoller.leftSampler.getBlackLine();
//				rightLineDetected = rightLineDetected || ColorPoller.rightSampler.getBlackLine();
//				boolean stopped = !leftMotor.isMoving() && !rightMotor.isMoving();
//				if (leftLineDetected && rightLineDetected || Math
//						.abs(ColorPoller.leftSampler.currentSample - ColorPoller.rightSampler.currentSample) < 0.06) {
//					Navigation.stopTheRobot();
//					break;
//				} else if (leftLineDetected) {
//					if (stopped) {
//						synchronized (rightMotor) {
//							synchronized (leftMotor) {
//								rightMotor.forward();
//							}
//						}
//					}
//					rightCounter++;
//					if (rightCounter > 15) {
//						synchronized (leftMotor) {
//							synchronized (rightMotor) {
//								Navigation.stopTheRobot();
//								return;
//							}
//						}
//					}
//				} else if (rightLineDetected) {
//					if (stopped) {
//						synchronized (leftMotor) {
//							synchronized (rightMotor) {
//								leftMotor.forward();
//							}
//						}
//					}
//					leftCounter++;
//					if (leftCounter > 15) {
//						synchronized (leftMotor) {
//							synchronized (rightMotor) {
//								Navigation.stopTheRobot();
//								return;
//							}
//						}
//					}
//				}
//			}
//			Main.sleepFor(20);
//		}
//	}
}
