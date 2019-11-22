package ca.mcgill.ecse211.finalproject.phase2;

import static ca.mcgill.ecse211.finalproject.Resources.ARENA_X;
import static ca.mcgill.ecse211.finalproject.Resources.ARENA_Y;
import static ca.mcgill.ecse211.finalproject.Resources.bin;
//TODO Remove after beta
//import static ca.mcgill.ecse211.finalproject.Resources.tnr;
import static ca.mcgill.ecse211.finalproject.Resources.green;
import static ca.mcgill.ecse211.finalproject.Resources.greenCorner;
import static ca.mcgill.ecse211.finalproject.Resources.island;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
import static ca.mcgill.ecse211.finalproject.Resources.tng;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Stack;

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
	 * current x-coordinate of the robot on the grid
	 */
	private static int currentX;
	/**
	 * current x-coordinate of the robot on the grid
	 */
	private static int currentY;

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
	public static int targetX;
	/**
	 * target Y coordinate
	 */
	public static int targetY;

	/**
	 * Contructor of the FathFinder
	 * 
	 * @param isRedTeam whether your team is the red team
	 */
	public PathFinder(boolean isRedTeam) {
// TODO Uncomment after beta
//		int corner = isRedTeam ? redCorner : greenCorner;
//		Region base = isRedTeam ? red : green;
//		Region tn = isRedTeam ? tnr : tng;

		int corner = greenCorner;
		Region base = green;
		Region tn = tng;

		System.out.println("Corner: " + corner);
		System.out.println("Base: " + base.toString());
		System.out.println("Tn: " + tn.toString());
		System.out.println("Island: " + island.toString());
		System.out.println("Arena X: " + ARENA_X + " Arena Y: " + ARENA_Y);

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
			navigation.xTile = 0;
			navigation.yTile = 0;
			Resources.odometer.setXYT(Resources.TILE_SIZE / 2d, Resources.TILE_SIZE / 2d, 0);
			break;
		case 1: // face west
			navigation.xTile = ARENA_X - 1;
			navigation.yTile = 0;
			Resources.odometer.setXYT(Resources.TILE_SIZE * (ARENA_X - 1 + 0.5), Resources.TILE_SIZE / 2d, 270);
			break;
		case 2: // face south
			navigation.xTile = ARENA_X - 1;
			navigation.yTile = ARENA_Y - 1;
			Resources.odometer.setXYT(Resources.TILE_SIZE * (ARENA_X - 1 + 0.5),
					Resources.TILE_SIZE * (ARENA_Y - 1 + 0.5), 180);
			break;
		case 3: // face east
			navigation.xTile = 0;
			navigation.yTile = ARENA_Y - 1;
			Resources.odometer.setXYT(Resources.TILE_SIZE / 2d, Resources.TILE_SIZE * (ARENA_Y - 1 + 0.5), 180);
			break;
		}

		// ====subject to change====//
		int[] launchCoords = findLaunchPointToTarget((int) bin.x, (int) bin.y);
		targetX = launchCoords[0];
		targetY = launchCoords[1];
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
	 * set the target square as an obstacle
	 * 
	 * @param x x coordinate of square
	 * @param y y coordinate of square
	 */
	public void setObstacle(int x, int y) {
		PathFinder.map[x][y].setStatus(-2);
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
		System.out.println(currentX);
		System.out.println(currentY);
		image[currentX][currentY] = 'C';
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
		int[] launchCoords = findLaunchPointToTarget((int) bin.x, (int) bin.y);
		targetX = launchCoords[0];
		targetY = launchCoords[1];
		
		open.add(map[currentX][currentY]);
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
				while (!ghettoStack.isEmpty()) {
					current = ghettoStack.pop();
					int[] entry = { current.X, current.Y };
					result.add(entry);
				}
				result.remove(0);
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
	 * ***** code copied over from lab 5 **** finds the square which the robot can
	 * launch from 6 squares away from the target
	 *
	 * @return an int array of size 3, which contains the x and y coordinates of the launch location, as well as the angle
	 * to turn to for launching the ball.
	 */
	private static int[] findLaunchPointToTarget(int binX, int binY) {
		int[] result = new int[3];
		double shortest_dist = 100;
		int[][] notableSquares = { { 0, 6 }, { 5, 5 }, { 6, 0 }, { 5, -5 }, { 0, -6 }, { -5, -5 }, { -6, 0 },
				{ -5, 5 } };
		int[] thetaOptions = { 180, 225, 270, 315, 0, 45, 90, 135 };
		for (int i = 0; i < notableSquares.length; i++) {
			int[] pair = notableSquares[i];
			boolean ooX = pair[0] + binX > ARENA_X || pair[0] + binX < 0;
			boolean ooY = pair[1] + binY > ARENA_Y || pair[1] + binY < 0;
			boolean invalid = map[pair[0]][pair[1]].status != 3;
			if (ooX || ooY || invalid) {
				continue;
			} else {
				double dist = Math
						.sqrt((navigation.xTile - (pair[0] + binX)) * (navigation.xTile - (pair[0] + binX)) +
								(navigation.yTile - (pair[1] + binY)) * (navigation.yTile - (pair[1] + binY)));
				if (dist < shortest_dist) {
					result[0] = pair[0] + binX;
					result[1] = pair[1] + binY;
					shortest_dist = dist;
					result[2] = (thetaOptions[i] + 360) % 360;
				}
			}
		}
		return result;
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
		 * base/tunnel 3 => central island 2 => tunnel 1 => base -2 => obstacle
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

	/**
	 * Test/debugger method
	 * 
	 * @param arenaSizeX
	 * @param arenaSizeY
	 * @param tunnelLLX
	 * @param tunnelLLY
	 * @param tunnelURX
	 * @param tunnelURY
	 * @param baseLLX
	 * @param baseLLY
	 * @param baseURX
	 * @param baseURY
	 * @param islandLLX
	 * @param islandLLY
	 * @param islandURX
	 * @param islandURY
	 * @param targetX
	 * @param targetY
	 * @return
	 */
	public static PathFinder test(boolean isRedTeam, int targetX, int targetY) {
		PathFinder pf = new PathFinder(isRedTeam);
		PathFinder.targetX = targetX;
		PathFinder.targetY = targetY;
		return pf;
	}

}
