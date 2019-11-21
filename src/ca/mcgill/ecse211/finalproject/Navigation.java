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
	 * target x coordinate of the current move being processed
	 */
	public int targetX = 0;

	/**
	 * target y coordinate of the current move being processed
	 */
	public int targetY = 0;

	/**
	 * whether the current move being processed in sucessful
	 */
	public boolean moveSuccessful = false;

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
	public int convertDistance(double distance) {
		return (int) ((180 * distance) / (Math.PI * WHEEL_RAD));
	}

	/**
	 * Converts input angle to the total rotation of each wheel needed to rotate the
	 * robot by that angle.
	 * 
	 * @param angle
	 * @return the wheel rotations necessary to rotate the robot by the angle
	 */
	public int convertAngle(double angle) {
		return convertDistance((Math.PI * TRACK * angle) / 360.0);
	}

	/**
	 * set Speed
	 */
	public void setSpeed(int speed) {
		synchronized (Resources.leftMotor) {
			synchronized (Resources.rightMotor) {
				leftMotor.setSpeed(speed);
				rightMotor.setSpeed(speed);
			}
		}
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
	public int[] findTarget(int targetX, int targetY) {
		int[] result = new int[3];
		double shortest_dist = 100;
		int[][] notableSquares = { { 0, 5 }, { 4, 4 }, { 5, 0 }, { 4, -4 }, { 0, -5 }, { -4, -4 }, { -5, 0 },
				{ -4, 4 } };
		int[] thetaOptions = { 180, 225, 270, 315, 0, 45, 90, 135 };
		for (int i = 0; i < notableSquares.length; i++) {
			int[] pair = notableSquares[i];
			boolean ooX = pair[0] + targetX > ARENA_X || pair[0] + targetX < 0;
			boolean ooY = pair[1] + targetY > ARENA_Y || pair[1] + targetY < 0;
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
	 * stops the robot in place
	 */
	public void stopTheRobot() {
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
	public boolean processNextMove(int[] move) {
		boolean result = true;
		targetX = move[0];
		targetY = move[1];
		switch (navigationMode) {
		case TRAVELING:
			goTo(targetX, targetY);
			break;
		case CORRECTING:
			Main.sleepFor(70); // maybe?
			break;
		case OBSTACLE_ENCOUNTERED:
			Resources.colorPoller.sleep();
			Sound.buzz();
			if (Resources.pathFinder.setObstacle()) {
				PathFinder.resetMap();
				Resources.pathFinder.findPath();
				result = false;
			}
			break;
		}
		if (moveSuccessful) {
			xTile = (int) (odometer.getXYT()[0] / TILE_SIZE);
			yTile = (int) (odometer.getXYT()[1] / TILE_SIZE);
		}
		return result;
	}

	/**
	 * turn and move to the center of a target tile
	 * 
	 * @param X x-coordinates of the target tile
	 * @param Y y-coordinates of the target tile
	 */
	public void goTo(int X, int Y) {
		
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
		
		Resources.colorPoller.sleep();
		synchronized (Resources.leftMotor) {
			synchronized (Resources.rightMotor) {
				leftMotor.rotate(convertAngle(angleDeviation), true);
				rightMotor.rotate(-convertAngle(angleDeviation), false);
			}
		}
		Resources.colorPoller.wake();
		Main.sleepFor(20);
		
		leftMotor.rotate(convertDistance(distance2go), true);
		rightMotor.rotate(convertDistance(distance2go), false);

		if (!interrupted) {
			moveSuccessful = true;
		}
		if (UltrasonicPoller.hasDetected()) {
			synchronized (navigationMode) {
				navigationMode = TravelingMode.OBSTACLE_ENCOUNTERED;
			}
		}
		Main.sleepFor(120);
	}

	/**
	 * turn to the target absolute theta
	 * 
	 * @param theta target angle to turn towards, 0 being facing North
	 */
	public void turnTo(double theta) {

		double angleDiff = theta - odometer.getXYT()[2];
		// Don't correct the angle if it is within a certain threshold
		if (Math.abs(angleDiff) < 3.0 || Math.abs(angleDiff) > 357.0) {
			return;
		}
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

	public void goToLowerLeftCorner() {
		turnTo(225);
		Resources.leftMotor.rotate(convertDistance(0.73 * TILE_SIZE), true);
		Resources.rightMotor.rotate(convertDistance(0.73 * TILE_SIZE), false);
	}
}
