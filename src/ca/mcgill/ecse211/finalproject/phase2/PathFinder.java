package ca.mcgill.ecse211.finalproject.phase2;

//import static ca.mcgill.ecse211.finalproject.Resources.ARENA_X;
//import static ca.mcgill.ecse211.finalproject.Resources.ARENA_Y;
//import static ca.mcgill.ecse211.finalproject.Resources.bin;
import static ca.mcgill.ecse211.finalproject.Resources.*;
//import static ca.mcgill.ecse211.finalproject.Resources.green;
//import static ca.mcgill.ecse211.finalproject.Resources.greenCorner;
//import static ca.mcgill.ecse211.finalproject.Resources.island;
//import static ca.mcgill.ecse211.finalproject.Resources.odometer;
//import static ca.mcgill.ecse211.finalproject.Resources.tng;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Stack;

import ca.mcgill.ecse211.finalproject.Navigation;
import ca.mcgill.ecse211.finalproject.Resources;
import ca.mcgill.ecse211.finalproject.Resources.Region;

/**
 * An implementation of the A* path finding algorithm which only allows cardinal
 * movements on the map
 * 
 * @author yp
 *
 */
public class PathFinder {

	/**
	 * orientation of the robot
	 * 
	 * @author yp
	 *
	 */

	/**
	 * a 2-D array used to represent the map
	 */
	private static Square[][] map;

	/**
	 * priority queue used in the A* algorithm
	 */
	private static PriorityQueue<Square> open = new PriorityQueue<Square>(20);
	/**
	 * array to record which squares on the map have been visited
	 */
	private static boolean[][] closed;
	/**
	 * current target X coordinate
	 */
	private static int targetX;
	/**
	 * target Y coordinate
	 */
	private static int targetY;
	/**
	 * list of at most 3 possible launch points
	 */
	public static ArrayList<double[]> launchPoints;

	private static int launchIndex = 0;

	public static double launchAngle;
	public static double launchAdjustment;
	private static boolean goingHome = false;

	/**
	 * Contructor of the FathFinder
	 * 
	 * @param isRedTeam whether your team is the red team
	 */
	public PathFinder(boolean isRedTeam) {
		int corner = isRedTeam ? redCorner : greenCorner;
		Region base = isRedTeam ? red : green;
		Region tn = isRedTeam ? tnr : tng;
		Point bin = isRedTeam ? redBin : greenBin;

//		System.out.println("Corner: " + greenCorner);
//		System.out.println("Base: " + green.toString());
//		System.out.println("Tn: " + tng.toString());
//		System.out.println("Island: " + island.toString());
//		System.out.println("Arena X: " + ARENA_X + " Arena Y: " + ARENA_Y);

		PathFinder.map = new Square[ARENA_X][ARENA_Y];
		PathFinder.closed = new boolean[ARENA_X][ARENA_Y];

		// set river
		for (int i = 0; i < ARENA_X; i++) {
			for (int j = 0; j < ARENA_Y; j++) {
				Square sq = new Square(i, j);
				map[i][j] = sq;
				sq.setStatus(0);
			}
		}

		// set base
		for (int i = (int) base.ll.x; i < (int) base.ur.x; i++) {
			for (int j = (int) base.ll.y; j < (int) base.ur.y; j++) {
				map[i][j].setStatus(1);
			}
		}

		// set island
		for (int i = (int) island.ll.x; i < (int) island.ur.x; i++) {
			for (int j = (int) island.ll.y; j < (int) island.ur.y; j++) {
				map[i][j].setStatus(3);
			}
		}
		// set tunnel
		for (int i = (int) tn.ll.x; i < (int) tn.ur.x; i++) {
			for (int j = (int) tn.ll.y; j < (int) tn.ur.y; j++) {
				map[i][j].setStatus(2);
			}
		}

		// set safeguard for tunnel, setting all tiles beside the tunnel to be river
		if ((int) tn.ur.x - (int) tn.ll.x > 1) { // long tunnel is horizontal
			int[] llCoord = { (int) tn.ll.x, (int) tn.ll.y };
			if (llCoord[1] - 1 >= 0 && llCoord[1] + 1 < ARENA_Y) {
				map[llCoord[0]][llCoord[1] - 1].setStatus(0);
				map[llCoord[0] + 1][llCoord[1] - 1].setStatus(0);
				map[llCoord[0]][llCoord[1] + 1].setStatus(0);
				map[llCoord[0] + 1][llCoord[1] + 1].setStatus(0);
			} else if (llCoord[1] - 1 >= 0) {
				map[llCoord[0]][llCoord[1] - 1].setStatus(0);
				map[llCoord[0] + 1][llCoord[1] - 1].setStatus(0);
			} else {
				map[llCoord[0]][llCoord[1] + 1].setStatus(0);
				map[llCoord[0] + 1][llCoord[1] + 1].setStatus(0);
			}
		} else if ((int) tn.ur.y - (int) tn.ll.y > 1) { // long tunnel is vertical
			int[] llCoord = { (int) tn.ll.x, (int) tn.ll.y };
			if (llCoord[0] - 1 >= 0 && llCoord[0] + 1 < ARENA_X) {
				map[llCoord[0] - 1][llCoord[1]].setStatus(0);
				map[llCoord[0] - 1][llCoord[1] + 1].setStatus(0);
				map[llCoord[0] + 1][llCoord[1]].setStatus(0);
				map[llCoord[0] + 1][llCoord[1] + 1].setStatus(0);
			} else if (llCoord[1] - 1 >= 0) {
				map[llCoord[0] - 1][llCoord[1]].setStatus(0);
				map[llCoord[0] - 1][llCoord[1] + 1].setStatus(0);
			} else {
				map[llCoord[0] + 1][llCoord[1]].setStatus(0);
				map[llCoord[0] + 1][llCoord[1] + 1].setStatus(0);
			}
		}

		switch (corner) {
		case 0: // face north
			Navigation.xTile = 0;
			Navigation.yTile = 0;
			Resources.odometer.setXYT(Resources.TILE_SIZE / 2d, Resources.TILE_SIZE / 2d, 0);
			break;
		case 1: // face west
			Navigation.xTile = ARENA_X - 1;
			Navigation.yTile = 0;
			Resources.odometer.setXYT(Resources.TILE_SIZE * (ARENA_X - 1 + 0.5), Resources.TILE_SIZE / 2d, 270);
			break;
		case 2: // face south
			Navigation.xTile = ARENA_X - 1;
			Navigation.yTile = ARENA_Y - 1;
			Resources.odometer.setXYT(Resources.TILE_SIZE * (ARENA_X - 1 + 0.5),
					Resources.TILE_SIZE * (ARENA_Y - 1 + 0.5), 180);
			break;
		case 3: // face east
			Navigation.xTile = 0;
			Navigation.yTile = ARENA_Y - 1;
			Resources.odometer.setXYT(Resources.TILE_SIZE / 2d, Resources.TILE_SIZE * (ARENA_Y - 1 + 0.5), 180);
			break;
		}

		launchPoints = findLaunchPointToTarget((int) bin.x, (int) bin.y);

		// ====subject to change====//
		targetX = (int) launchPoints.get(launchIndex)[0];
		targetY = (int) launchPoints.get(launchIndex)[1];
		launchAngle = launchPoints.get(launchIndex)[2];
		launchAdjustment = launchPoints.get(launchIndex)[3];
	}

	/**
	 * resets the map so that a new round of path finding can be run
	 */
	public static void resetMap() {
		for (Square[] row : map) {
			for (Square sq : row) {
				sq.hCost = 0;
				sq.finalCost = 0;
				sq.parent = null;
			}
		}
		open = new PriorityQueue<Square>();
		PathFinder.closed = new boolean[map.length][map[0].length];
	}

	/**
	 * set the target square in front of the robot as an obstacle
	 * 
	 * @param x x coordinate of square
	 * @param y y coordinate of square
	 */
	public boolean setObstacle() {
		if (!isFacingAWall()) {
			double[] currXYT = odometer.getXYT();
			if (currXYT[2] >= 45 && currXYT[2] < 135) {// facing EAST
				map[Navigation.xTile + 1][Navigation.yTile].setStatus(-2);
				if (!goingHome && map[Navigation.xTile + 1][Navigation.yTile].X == targetX
						&& map[Navigation.xTile + 1][Navigation.yTile].Y == targetY) {
					resetLaunchPoint();
				}
				return true;
			} else if (currXYT[2] >= 135 && currXYT[2] < 225) {// facing SOUTH
				map[Navigation.xTile][Navigation.yTile - 1].setStatus(-2);
				if (!goingHome && map[Navigation.xTile][Navigation.yTile - 1].X == targetX
						&& map[Navigation.xTile][Navigation.yTile - 1].Y == targetY) {
					resetLaunchPoint();
				}
				return true;
			} else if (currXYT[2] >= 225 && currXYT[2] < 315) {// facing WEST
				map[Navigation.xTile - 1][Navigation.yTile].setStatus(-2);
				if (!goingHome && map[Navigation.xTile - 1][Navigation.yTile].X == targetX
						&& map[Navigation.xTile - 1][Navigation.yTile].Y == targetY) {
					resetLaunchPoint();
				}
				return true;
			} else {
				map[Navigation.xTile][Navigation.yTile + 1].setStatus(-2);
				if (!goingHome && map[Navigation.xTile][Navigation.yTile + 1].X == targetX
						&& map[Navigation.xTile][Navigation.yTile + 1].Y == targetY) {
					resetLaunchPoint();
				}
				return true;
			}
		}
		return false;

	}

	/**
	 * this checks the heuristic cost of an adjacent square, updates the final cost
	 * and adds it into the priority queue
	 * 
	 * @param current current square probing from
	 * @param target  square being probed
	 * @param cost    cost increment of the target
	 */
	public void checkAndUpdateCost(Square current, Square target, double cost) {
		if (target.status <= 0 || closed[target.X][target.Y]) {
			return;
		}
		double t_final_cost = target.hCost + cost;
		boolean inOpen = open.contains(target);
		if (!inOpen || t_final_cost < target.finalCost) {
			target.finalCost = t_final_cost;
			target.parent = current;
			if (!inOpen)
				open.add(target);
		}
	}

	/**
	 * prints the map for debugging
	 */
	public void printMap() {
		char[][] image = new char[map.length][map[0].length];
		for (Square[] row : map) {
			for (Square sq : row) {
				if (sq.status == 0) {
					image[sq.X][sq.Y] = '~';
				} else if (sq.status == 1) {
					image[sq.X][sq.Y] = 'B';
				} else if (sq.status == 2) {
					image[sq.X][sq.Y] = 'U';
				} else if (sq.status == -2) {
					image[sq.X][sq.Y] = 'X';
				} else if (sq.status == 3) {
					image[sq.X][sq.Y] = 'O';
				}
			}
		}
		image[Navigation.xTile][Navigation.yTile] = 'C';
		image[targetX][targetY] = 'T';
		for (char[] row : image) {
			System.out.println(Arrays.toString(row));
		}
	}

	/**
	 * runs the A* algorithm and finds the shortest path to target using only
	 * cardinal movements
	 * 
	 * @return a list of int[] of size 2, which represents the list of squares the
	 *         robot should move to in order to reach to the target
	 */
	public ArrayList<int[]> findPath() {

		open.add(map[Navigation.xTile][Navigation.yTile]);
		Square current;
		while (true) {
			current = open.poll();
			if (current == null) {
				System.out.println("Something went horribly wrong, no path found");
				break;
			}
			closed[current.X][current.Y] = true;

			if (current.equals(map[targetX][targetY])) {
				ArrayList<int[]> result = new ArrayList<int[]>();
				// we use a Stack to help reverse the linked list and calculate the movement
				// (instead of returning squares)
				Stack<Square> ghettoStack = new Stack<Square>();
				ghettoStack.push(current);
				while (current.parent != null) {
					ghettoStack.push(current.parent);
					current = current.parent;
				}

				ghettoStack.pop(); // this is the current square I know what I'm doing I think

				while (!ghettoStack.isEmpty()) {
					current = ghettoStack.pop();
					int[] entry = { current.X, current.Y };
					result.add(entry);
				}
				return result;
			}

			Square targetSquare;
			if (current.X - 1 >= 0) {
				targetSquare = map[current.X - 1][current.Y]; // WEST
				targetSquare.hCost = current.getDistanceTo(map[targetX][targetY]);
				checkAndUpdateCost(current, targetSquare, current.finalCost + 1);
			}

			if (current.Y - 1 >= 0) {
				targetSquare = map[current.X][current.Y - 1]; // SOUTH
				targetSquare.hCost = current.getDistanceTo(map[targetX][targetY]);
				checkAndUpdateCost(current, targetSquare, current.finalCost + 1);
			}

			if (current.Y + 1 < map[0].length) {
				targetSquare = map[current.X][current.Y + 1]; // NORTH
				targetSquare.hCost = current.getDistanceTo(map[targetX][targetY]);
				checkAndUpdateCost(current, targetSquare, current.finalCost + 1);
			}

			if (current.X + 1 < map.length) {
				targetSquare = map[current.X + 1][current.Y]; // EAST
				targetSquare.hCost = current.getDistanceTo(map[targetX][targetY]);
				checkAndUpdateCost(current, targetSquare, current.finalCost + 1);
			}
		}
		return null;
	}

	/**
	 * 
	 * @param targetX
	 * @param targetY
	 * @return
	 */
	private static ArrayList<double[]> findLaunchPointToTarget(int targetX, int targetY) {
		ArrayList<double[]> listOfTargets = new ArrayList<double[]>();
		double shortest_dist = Double.MAX_VALUE;
		double[][] notableSquares = { { 0, 6 }, { 1, 6 }, { 3, 5 }, { 4, 4 }, { 5, 3 }, { 6, 1 }, { 6, 0 }, { 6, -1 },
				{ 5, -3 }, { 4, -4 }, { 3, -5 }, { 1, -6 }, { 0, 6 }, { -1, -6 }, { -3, -5 }, { -4, -4 }, { -5, -3 },
				{ -6, -1 }, { -6, 0 }, { -6, 1 }, { -5, 3 }, { -4, 4 }, { -3, 5 }, { -1, 6 } };
		double[] thetaOptions = { 180, 189.5, 211, 225, 239, 260.5, 270, 279.5, 301, 315, 329, 350.5, 0, 9.5, 31, 45,
				59, 80.5, 90, 99.5, 121, 135, 149, 170.5 };
		double[] distanceAdjustment = { 0, 0.083, -0.169, -0.344, -0.169, 0.083, 0, 0.083, -0.169, -0.344, -0.169,
				0.083, 0, 0.083, -0.169, -0.344, -0.169, 0.083, 0, 0.083, -0.169, -0.344, -0.169, 0.083 };
		for (int i = 0; i < notableSquares.length; i++) {
			double[] pair = notableSquares[i];
			boolean ooX = pair[0] + targetX >= ARENA_X || pair[0] + targetX - 1 <= 0; // -1 to prevent running into
																						// wall,
			boolean ooY = pair[1] + targetY >= ARENA_Y || pair[1] + targetY - 1 <= 0;
			boolean invalid;
			if (!ooX && !ooY) {
				invalid = map[(int) pair[0] + targetX][(int) pair[1] + targetY].status != 3;
			} else {
				invalid = true;
			}

			if (ooX || ooY || invalid) {
				continue;
			} else {
				double[] result = new double[4];
				double dist = Math
						.sqrt((pair[0] + targetX) * (pair[0] + targetX) + (pair[1] + targetY) * (pair[1] + targetY));
				result[0] = pair[0] + targetX;
				result[1] = pair[1] + targetY;
				shortest_dist = dist;
				result[2] = thetaOptions[i];
				result[3] = distanceAdjustment[i];
				if (dist < shortest_dist) {
					listOfTargets.add(0, result);
				} else {
					listOfTargets.add(result);
				}
			}
		}
		return listOfTargets;
	}

	/**
	 * class used to represent the squares used in the map for A* path finding
	 * 
	 * @author yp
	 *
	 */
	private class Square implements Comparable<Square> {
		/**
		 * heuristic cost for the square to get to target, calculated by the Euclidean
		 * distance
		 */
		double hCost = 0;
		/**
		 * final cost for the target square to get to the target. sum of hCost and the
		 * number of square traveled
		 */
		double finalCost = 0;
		/**
		 * X coordinate of the square
		 */
		int X;
		/**
		 * Y coordinate of the sqaure
		 */
		int Y;
		/**
		 * number represents the type of terrain of the sqaure 0 => river or enemy
		 * base/tunnel 3 => central island 2 => tunnel 1 => base -2=> obstacle
		 */
		int status;
		/**
		 * previous square on the path when searching
		 */
		Square parent;

		/**
		 * constructor for a single square on the map
		 * 
		 * @param x X coordinate of the square
		 * @param y Y coordinate of the square
		 */
		Square(int x, int y) {
			this.X = x;
			this.Y = y;
			this.status = 0;
		}

		/**
		 * returns the Eulidean distance from one square to another
		 * 
		 * @param anotherone another way point
		 * @return
		 */
		double getDistanceTo(Square anotherone) {
			return Math.sqrt((this.X - anotherone.X) * (this.X - anotherone.X)
					+ (this.Y - anotherone.Y) * (this.Y - anotherone.Y));
		}

		/**
		 * sets the values for the status of the square, each number represents a type
		 * of terrain 0 => river or enemy base/tunnel 3 => central island 2 => tunnel 1
		 * => base -2=> obstacle
		 */
		void setStatus(int status) {
			this.status = status;
		}

		/**
		 * used for the compare the cost in the priority queue
		 */
		@Override
		public int compareTo(Square c) {
			return this.finalCost < c.finalCost ? -1 : this.finalCost > c.finalCost ? 1 : 0;
		}

		/**
		 * prints the square to help debug
		 */
		@Override
		public String toString() {
			return ("[" + X + "," + Y + "]");
		}

	}

	public static void setTarget(int x, int y) {
		targetX = x;
		targetY = y;
	}

	public static boolean isFacingAWall() {

		double[] currXYT = odometer.getXYT();
		if (currXYT[2] >= 45 && currXYT[2] < 135) {// facing EAST
			if (Navigation.xTile + 1 > Resources.ARENA_X - 1) {
				return true;
			}
		} else if (currXYT[2] >= 135 && currXYT[2] < 225) {// facing SOUTH
			if (Navigation.yTile - 1 < 0) {
				return true;
			}
		} else if (currXYT[2] >= 225 && currXYT[2] < 315) {// facing WEST
			if (Navigation.xTile - 1 < 0) {
				return true;
			}
		} else {
			if (Navigation.yTile + 1 > Resources.ARENA_Y - 1) {
				return true;
			}
		}
		return false;
	}

	public static void resetLaunchPoint() {
		launchIndex++;
		targetX = (int) launchPoints.get(launchIndex)[0];
		targetY = (int) launchPoints.get(launchIndex)[1];
		while (isAdjacentToObstacle (targetX,targetY)) {
			launchIndex++;
			targetX = (int) launchPoints.get(launchIndex)[0];
			targetY = (int) launchPoints.get(launchIndex)[1];
		}
		launchAngle = launchPoints.get(launchIndex)[2];
		launchAdjustment = launchPoints.get(launchIndex)[3];
	}

	public static void letsGoHome() {
		// Picks corner based on which team we are
		switch (Resources.TEAM_NUMBER == Resources.redTeam ? redCorner : greenCorner) {
		case 0: // face north
			targetX = 0;
			targetY = 0;
			break;
		case 1: // face west
			targetX = ARENA_X - 1;
			targetY = 0;
			break;
		case 2: // face south
			targetX = ARENA_X - 1;
			targetY = ARENA_Y - 1;
			break;
		case 3: // face east
			targetX = 0;
			targetY = ARENA_Y - 1;
			break;
		}

	}
	
	public static boolean isAdjacentToObstacle (int x, int y) {
		if (x == 0 || y == 0) {
			return true;
		}
		else {
			return ((map[x-1][y].status == -2)||(map[x][y-1].status == -2)||(map[x-1][y-1].status == -2));
		}
	}
	
}
