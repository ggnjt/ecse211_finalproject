package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.ACCELERATION;
import static ca.mcgill.ecse211.finalproject.Resources.ARENA_X;
import static ca.mcgill.ecse211.finalproject.Resources.ARENA_Y;
import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.TRACK;
import static ca.mcgill.ecse211.finalproject.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.finalproject.Resources.leftMotor;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.rightMotor;

import java.util.ArrayList;
import java.util.Arrays;

import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
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
	public enum TravelingMode {
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
	 * Constructor for the Navigation class.
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
	 * set Speed
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
		double distance2go = Math.sqrt(X2go * X2go + Y2go * Y2go);

		int speed = Resources.leftMotor.getSpeed();

		UltrasonicPoller.sleep();
		Resources.colorPoller.sleep();

		synchronized (Resources.leftMotor) {
			synchronized (Resources.rightMotor) {
				setSpeed(Resources.ROTATE_SPEED);
				leftMotor.rotate(convertAngle(angleDeviation), true);
				rightMotor.rotate(-convertAngle(angleDeviation), false);
				setSpeed(speed);
			}
		}
		Resources.colorPoller.wake();
		UltrasonicPoller.wake();
		// reset to get more accurate reading

		if (Math.abs(angleDeviation) > 60) {
			UltrasonicPoller.resetDetection();
			Main.sleepFor(800);
		} else {
			Main.sleepFor(250);
		}
		if (!PathFinder.isFacingAWall() && UltrasonicPoller.hasDetected()) {
			moveSuccessful = false;
			interrupted = true;
			synchronized (navigationMode) {
				navigationMode = TravelingMode.OBSTACLE_ENCOUNTERED;
				return;
			}
		}

		leftMotor.rotate(convertDistance(distance2go), true);
		rightMotor.rotate(convertDistance(distance2go), false);

		if (!interrupted) {
			moveSuccessful = true;
		}
		if (moveSuccessful) {
			xTile = (int) (odometer.getXYT()[0] / TILE_SIZE);
			yTile = (int) (odometer.getXYT()[1] / TILE_SIZE);
		}
		if (!PathFinder.isFacingAWall() && UltrasonicPoller.hasDetected()) {
			synchronized (navigationMode) {
				navigationMode = TravelingMode.OBSTACLE_ENCOUNTERED;
			}
		}
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
		// Don't correct the angle if it is within a certain threshold
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

	public static void goToLowerLeftCorner() {
		Resources.colorPoller.sleep();
		turnTo(225);
		synchronized (leftMotor) {
			synchronized (rightMotor) {
				Resources.leftMotor.rotate(convertDistance(0.73 * TILE_SIZE), true);
				Resources.rightMotor.rotate(convertDistance(0.73 * TILE_SIZE), false);
			}
		}
	}
	
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
		Resources.colorPoller.wake();
	}

	public static boolean run(ArrayList<int[]> moves) {
		for (int[] move : moves) {
			System.out.println(Arrays.toString(move));
			UltrasonicPoller.resetDetection();
			setSpeed(Resources.LOW_FORWARD_SPEED);
			processNextMove(move);
			while (!moveSuccessful || interrupted) {
				UltrasonicPoller.resetDetection();
				if (navigationMode == TravelingMode.TRAVELING) {
					if (targetY == 4) {
						Resources.colorPoller.sleep();
					}
					processNextMove(move);
				} else if (navigationMode == TravelingMode.OBSTACLE_ENCOUNTERED) {
					UltrasonicPoller.resetDetection();
					break;
				} else {
					Main.sleepFor(70);
				}
				Resources.colorPoller.wake();
			}
			if (navigationMode == TravelingMode.OBSTACLE_ENCOUNTERED) {
				if (Resources.pathFinder.setObstacle()) {
					PathFinder.resetMap();
					Main.moves = Resources.pathFinder.findPath();
					navigationMode = TravelingMode.TRAVELING;
					moveSuccessful = false;
					Navigation.interrupted = true;
					return false;
				}
			}
		}
		return true;
	}
	
	public static void moveByDistance(double distance) {
		synchronized (leftMotor) {
			synchronized (rightMotor) {
				Resources.leftMotor.rotate(convertDistance(distance), true);
				Resources.rightMotor.rotate(convertDistance(distance), false);
			}
		}
	}
}
