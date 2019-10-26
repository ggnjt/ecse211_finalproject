package ca.mcgill.ecse211.finalproject;

import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.Stack;


public class PathFinder {

	public enum Orientation {
		NORTH, SOUTH, EAST, WEST
	}

	static private Square[][] map;
	
	/**
	 * current position and orientation of the robot on the grid
	 */
	private static int currentX;
	private static int currentY;
	private Orientation orientation;

	/**
	 * A* stuff
	 */
	private static PriorityQueue<Square> open = new PriorityQueue<Square>();
	private static boolean[][] closed;
	private static int targetX;
	private static int targetY;



	public PathFinder(int arenaSizeX, int arenaSizeY, int tunnelLLX, int tunnelLLY, int tunnelURX, int tunnelURY,
			int baseLLX, int baseLLY, int baseURX, int baseURY, int islandLLX, int islandLLY, int islandURX,
			int islandURY) {

		PathFinder.map = new Square[arenaSizeX][arenaSizeX];

		for (int i = baseLLX; i < baseURX; i++) {
			for (int j = baseLLY; j < baseURY; j++) {
				Square sq = new Square(i, j);
				map[i][j] = sq;
				sq.setStatus(-1);
			}
		}
		for (int i = tunnelLLX; i < tunnelURX; i++) {
			for (int j = tunnelLLY; j < tunnelURY; j++) {
				Square sq = new Square(i, j);
				map[i][j] = sq;
				sq.setStatus(-2);
			}
		}
		for (int i = islandLLX; i < islandURX; i++) {
			for (int j = islandLLY; j < islandURY; j++) {
				Square sq = new Square(i, j);
				map[i][j] = sq;
				sq.setStatus(1);
			}
		}

		this.currentX = 0;
		this.currentY = 0;
		this.orientation = Orientation.NORTH;
	}
	
	
	/**
	 * resets the map so that a new round of path finding can be run
	 */
	public void resetMap() {
		for (Square [] row: map) {
			for (Square sq : row) {
				sq.hCost = 0;
				sq.finalCost = 0;
				sq.parent = null;
			}
		}
		open = new PriorityQueue<Square>();
	}
	
	
	/**
	 * Add the square right in front of the robot as an obstacle
	 */
	public void addObstacle() {
		//TODO
		switch (this.orientation) {
		case NORTH:
		case SOUTH:
		case EAST:
		case WEST:
		}
	}
	
	public void checkAndUpdateCost(Square current, Square target, double cost) {
		if(target == null || closed[target.X][target.Y]) {
			return;
		}
        double t_final_cost = target.hCost+cost;
        boolean inOpen = open.contains(target);
        if(!inOpen || t_final_cost<target.finalCost){
            target.finalCost = t_final_cost;
            target.parent = current;
            if(!inOpen)open.add(target);
        }
	}
	
	/**
	 * after running this method, the target square should have a path marked with parent
	 */
	public ArrayList<int[]> findPath () {
		open.add(map[currentX][currentY]);
		Square current;
		
		while(true){ 
            current = open.poll();
            if(current==null) {
            	System.out.println("Something went horribly wrong");
            	break;
            }
            current.hCost = current.getDistanceTo(map[targetX][targetY]);
            closed[current.X][current.Y]=true; 

            if(current.equals(map[targetX][targetY])){
            	ArrayList<int[]> result = new ArrayList<int[]>();
                Stack<Square> ghettoStack = new Stack<Square>();
                ghettoStack.push(current);
                while (current.parent != null) {
                	ghettoStack.push(current.parent);
                	current = current.parent;
                }
                current = ghettoStack.pop();
                while (!ghettoStack.isEmpty()) {
                	int [] entry = {ghettoStack.peek().X - current.X ,ghettoStack.peek().Y - current.Y};
                	result.add(entry);
                	current = ghettoStack.pop();
                }
                int [] entry = {targetX - current.X ,targetY - current.Y};
            	result.add(entry);
            	return result;
            } 
            
            Square t;  
            if(current.X-1>=0){
                t = map[current.X-1][current.Y];
                checkAndUpdateCost(current, t, current.finalCost + 1); 
            } 

            if(current.Y-1>=0){
                t = map[current.X][current.Y-1];
                checkAndUpdateCost(current, t, current.finalCost + 1); 
            }

            if(current.Y+1<map[0].length){
                t = map[current.X][current.Y+1];
                checkAndUpdateCost(current, t, current.finalCost + 1); 
            }

            if(current.X+1<map.length){
                t = map[current.X+1][current.Y];
                checkAndUpdateCost(current, t, current.finalCost + 1); 
            }
        }
		System.out.println("something went horribly wrong");
		return null;
	}

	// ***** code copied over from lab 5 **** //
	private static int[][] getListOfMovesToTarget(int[] target) {
		int[][] result;
		boolean xBigger;
		if (target[0] == 0 || target[1] == 0) {
			result = new int[1][2];
			result[0] = target;
		} else {
			int stepSize;
			xBigger = target[0] > target[1];
			result = new int[(xBigger ? target[1] : target[0])][2];
			if (xBigger) { // per y step, x will move by stepSize
				stepSize = target[0] / target[1];
				int i = 0, j = 0;
				while (j < target[1] - 1) {
					result[j] = new int[] { stepSize, 1 };
					i += stepSize;
					j++;
				}
				result[j] = new int[] { target[0] - i, 1 };
				return result;
			} else { // per x step, y will move by stepSize
				stepSize = target[1] / target[0];
				int i = 0, j = 0;
				while (j < target[0] - 1) {
					result[j] = new int[] { 1, stepSize };
					i += stepSize;
					j++;
				}
				result[j] = new int[] { 1, target[1] - i };
				return result;
			}
		}
		return result;
	}

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
	 * class for squares on the map used for A* algorithm
	 * 
	 * @author yp
	 */
	private class Square {
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
		 * returns the distance needed to travel by cardinal directions from one
		 * waypoint to another
		 * 
		 * @param anotherone another way point
		 * @return
		 */
		double getDistanceTo(Square anotherone) {
			return Math.sqrt((this.X - anotherone.X) * (this.X - anotherone.X)
					+ (this.Y - anotherone.Y) * (this.Y - anotherone.Y));
		}

		@Override
        public String toString(){
            return "["+this.X+", "+this.Y+"]";
        }
		
		boolean equals(Square anotherone) {
			return this.X == anotherone.X && this.Y == anotherone.Y;
		}

		/**
		 * 0 => river or enemy base/tunnel 1 => central island -1 => base -2 => tunnel
		 * -3 => obstacle
		 */
		void setStatus(int status) {
			this.status = status;
		}

	}

}
