package ca.mcgill.ecse211.finalproject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Stack;

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
	static private Square[][] map;

	/**
	 * current position and orientation of the robot on the grid
	 */
	private static int currentX;
	private static int currentY;

	/**
	 * A* stuff
	 */
	private static PriorityQueue<Square> open = new PriorityQueue<Square>(20); //priority queue used to determine the best path
	private static boolean[][] closed; //array to record which squares have been visited
	private static int targetX; //target X coordinate
	private static int targetY; //target Y coordinate
	
	/**
	 * constructor which initialized the map and the path finder. the enemy tunnel
	 * and base are not considered and regarded as the same as the river this should
	 * be called after localization
	 * 
	 * @param arenaSizeX arena x size -1
	 * @param arenaSizeY arena y size -1
	 * @param tunnelLLX  lower left corner X coordinate of OUR tunnel
	 * @param tunnelLLY  lower left corner Y coordinate of OUR tunnel
	 * @param tunnelURX  upper right corner X coordinate of OUR tunnel
	 * @param tunnelURY  upper right corner Y coordinate of OUR tunnel
	 * @param baseLLX    lower left corner X coordinate of OUR base
	 * @param baseLLY    lower left corner Y coordinate of OUR base
	 * @param baseURX    upper right corner X coordinate of OUR base
	 * @param baseURY    upper right corner Y coordinate of OUR base
	 * @param islandLLX  lower left corner X coordinate of the central island
	 * @param islandLLY  lower left corner Y coordinate of the central island
	 * @param islandURX  upper right corner X coordinate of the central island
	 * @param islandURY  upper right corner Y coordinate of the central island
	 */
	public PathFinder(int arenaSizeX, int arenaSizeY, int tunnelLLX, int tunnelLLY, int tunnelURX, int tunnelURY,
			int baseLLX, int baseLLY, int baseURX, int baseURY, int islandLLX, int islandLLY, int islandURX,
			int islandURY) {

		PathFinder.map = new Square[arenaSizeX][arenaSizeY];
		PathFinder.closed = new boolean[arenaSizeX][arenaSizeY];

		// set river
		for (int i = 0; i < arenaSizeX; i++) {
			for (int j = 0; j < arenaSizeY; j++) {
				Square sq = new Square(i, j);
				map[i][j] = sq;
				sq.setStatus(0);
			}
		}

		// set base
		for (int i = baseLLX; i < baseURX; i++) {
			for (int j = baseLLY; j < baseURY; j++) {
				map[i][j].setStatus(1);
			}
		}
		// set tunnel
		for (int i = tunnelLLX; i < tunnelURX; i++) {
			for (int j = tunnelLLY; j < tunnelURY; j++) {
				map[i][j].setStatus(2);
			}
		}
		// set island
		for (int i = islandLLX; i < islandURX; i++) {
			for (int j = islandLLY; j < islandURY; j++) {
				map[i][j].setStatus(3);
			}
		}

		// TODO: subject to change
		PathFinder.currentX = 0;
		PathFinder.currentY = 8;
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
	 * Add the square right in front of the robot as an obstacle
	 */
	public void addObstacle() {
		// TODO
		
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
	 * prints the map
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
		image[currentX][currentY] = 'C';
		image[targetX][targetY] = 'T';
		for (char[] row : image) {
			System.out.println(Arrays.toString(row));
		}
	}

	/**
	 * this returns a list of int[] of size 2, which represents the list of moves
	 * the robot should make to get to the target
	 * 
	 * @return
	 */
	public ArrayList<int[]> findPath() {
		
		open.add(map[currentX][currentY]);
		Square current;

		while (true) {
			current = open.poll();
			if (current == null) {
				System.out.println("Something went horribly wrong");
				break;
			}
			closed[current.X][current.Y] = true;

			if (current.equals(map[targetX][targetY])) {
				// at this point, you are at a target and the route can be traced back by using
				// the parent Sqaures
				ArrayList<int[]> result = new ArrayList<int[]>();
				// we use a Stack to help reverse the linked list and calculate the movement
				// (instead of iusing squares)
				Stack<Square> ghettoStack = new Stack<Square>();
				ghettoStack.push(current);
				while (current.parent != null) {
					ghettoStack.push(current.parent);
					current = current.parent;
				}
				current = ghettoStack.pop();
				while (!ghettoStack.isEmpty()) {
					int[] entry = { ghettoStack.peek().X - current.X, ghettoStack.peek().Y - current.Y };
					result.add(entry);
					current = ghettoStack.pop();
				}
				return result;
			}

			Square t;
			if (current.X - 1 >= 0) {
				t = map[current.X - 1][current.Y];
				t.hCost = current.getDistanceTo(map[targetX][targetY]);
				checkAndUpdateCost(current, t, current.finalCost + 1);
			}

			if (current.Y - 1 >= 0) {
				t = map[current.X][current.Y - 1];
				t.hCost = current.getDistanceTo(map[targetX][targetY]);
				checkAndUpdateCost(current, t, current.finalCost + 1);
			}

			if (current.Y + 1 < map[0].length) {
				t = map[current.X][current.Y + 1];
				t.hCost = current.getDistanceTo(map[targetX][targetY]);
				checkAndUpdateCost(current, t, current.finalCost + 1);
			}

			if (current.X + 1 < map.length) {
				t = map[current.X + 1][current.Y];
				t.hCost = current.getDistanceTo(map[targetX][targetY]);
				checkAndUpdateCost(current, t, current.finalCost + 1);
			}
		}
		return null;
	}

	// ***** code copied over from lab 5 **** //
	private static int[] findLaunchPointToTarget(int targetX, int targetY) {

		int[] result = new int[2];
		double shortest_dist = 100;
		int[][] notableSquares = { { -3, 3 }, { 0, 4 }, { 3, 3 }, { -4, 0 }, { 4, 0 }, { -3, -3 }, { 0, -4 },
				{ 3, -3 } };
		for (int[] pair : notableSquares) {
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
		double hCost = 0;
		double finalCost = 0;
		int X;
		int Y;
		int status;
		Square parent;

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
		 * sets the values for the status of the square, each number represents a type of terrain
		 * 0 => river or enemy base/tunnel 
		 * 3 => central island 
		 * 2 => tunnel 
		 * 1 => base 
		 * -2=> obstacle
		 */
		void setStatus(int status) {
			this.status = status;
		}

		@Override
		public int compareTo(Square c) {
			return this.finalCost < c.finalCost ? -1 : this.finalCost > c.finalCost ? 1 : 0;
		}
		
		/**
		 * prints the square to help debug
		 */
		public String toString(){
			return ("["+X+","+Y+"]");
		}

	}

	
	// ========== Tester/debugger methods ==========//
	public static PathFinder test(int arenaSizeX, int arenaSizeY, int tunnelLLX, int tunnelLLY, int tunnelURX,
			int tunnelURY, int baseLLX, int baseLLY, int baseURX, int baseURY, int islandLLX, int islandLLY,
			int islandURX, int islandURY, int targetX, int targetY) {
		PathFinder pf = new PathFinder(arenaSizeX, arenaSizeY, tunnelLLX, tunnelLLY, tunnelURX, tunnelURY, baseLLX,
				baseLLY, baseURX, baseURY, islandLLX, islandLLY, islandURX, islandURY);
		PathFinder.targetX = targetX;
		PathFinder.targetY = targetY;
		return pf;
	}
	
	public static void main(String[] args) {
		
		PathFinder pf = PathFinder.test(15, 9, 4, 7, 6, 8, 0, 5, 4, 9, 6, 5, 15, 9, 12, 6);
		pf.setObstacle(7, 6);
		pf.setObstacle(7, 7);
		pf.setObstacle(7, 8);
		pf.setObstacle(2, 6);
		pf.setObstacle(2, 7);
		pf.setObstacle(2, 8);
		pf.printMap();

		ArrayList<int[]> lel = pf.findPath();
		
		for (int[] lol : lel) {
			System.out.println(Arrays.toString(lol));
		}
		
		

		PathFinder.resetMap();
		pf.printMap();

		lel = pf.findPath();
		for (int[] lol : lel) {
			System.out.println(Arrays.toString(lol));
		}
		
	}
	
	

}
