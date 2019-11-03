package ca.mcgill.ecse211.finalproject;

import java.util.ArrayList;


public class PathFinder {
	
	public enum Orientation {
		NORTH,SOUTH,EAST,WEST
	}
	
	/**
	 * 0	=> 	river or enemy base/tunnel
	 * 1	=> 	central island
	 * -1 	=>	base
	 * -2	=>	tunnel
	 * -3	=>	obstacle
	 */
	static private int[][] map;
	private int Xpos;
	private int YPos;
	private Orientation orientation;
	private ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
	// list[1] and list [2] should always be the two waypoints of the tunnel
	// list[0] is the origin point

	public PathFinder(int arenaSizeX, int arenaSizeY, int tunnelLLX, int tunnelLLY, int tunnelURX, int tunnelURY,
			int baseLLX, int baseLLY, int baseURX, int baseURY, int islandLLX, int islandLLY, int islandURX,
			int islandURY, boolean isTunnelVertical) {

		this.map = new int[arenaSizeX][arenaSizeX];
		
		for (int i = baseLLX; i < baseURX; i++) {
			for (int j = baseLLY; j < baseURY; j++) {
				map [i][j] = -1;
			}
		}
		for (int i = tunnelLLX; i < tunnelURX; i++) {
			for (int j = tunnelLLY; j < tunnelURY; j++) {
				map [i][j] = -2;
			}
		}
		for (int i = islandLLX; i < islandURX; i++) {
			for (int j = islandLLY; j < islandURY; j++) {
				map [i][j] = 1;
			}
		} 
		
		waypoints.add(new Waypoint (0,0));
		
		if (isTunnelVertical) {
			waypoints.add(new Waypoint(tunnelLLX, tunnelLLY-1));
			waypoints.add(new Waypoint (tunnelURX-1, tunnelURY));
		} else {
			waypoints.add(new Waypoint(tunnelLLX-1, tunnelLLY));
			waypoints.add(new Waypoint (tunnelURX, tunnelURY-1));
		}
		
		this.Xpos = 0;
		this.YPos = 0;
		this.orientation = Orientation.NORTH;
	}
	
	public void addObstacle() {
		switch (this.orientation) {
		case NORTH:
			map[this.Xpos][this.YPos+1] = -3;
		case SOUTH:
		case EAST:
		case WEST: 
		}
	}
	 
	// ***** code copied over from lab 5 **** //
	private static int [][] getListOfMovesToTarget(int[] target){
		int [][] result;
		boolean xBigger;
		if (target[0] == 0 || target [1] == 0) {
			result = new int [1][2];
			result[0] = target;
		}
		else {
			int stepSize;
			xBigger = target [0] > target[1];
			result = new int [(xBigger? target[1]:target[0])][2];
			if (xBigger) { //per y step, x will move by stepSize
				stepSize = target[0] / target[1];
				int i = 0,j = 0;
				while (j < target[1]-1) {
					result[j] = new int [] {stepSize,1};
					i+= stepSize;
					j++;
				}
				result[j] = new int [] {target[0] - i,1};
				return result;
			}
			else { //per x step, y will move by stepSize
				stepSize = target[1] / target[0];
				int i = 0,j = 0;
				while (j < target[0]-1) {
					result[j] = new int [] {1,stepSize};
					i+= stepSize;
					j++;
				}
				result[j] = new int [] {1,target[1] - i};
				return result;
			}
		}
		return result;
	}

	private static int [] findLaunchPointToTarget(int targetX, int targetY){
		
		int [] result = new int [2];
		double shortest_dist = 100;
		int [][] notableSquares = {{-3,3},{0,4},{3,3},{-4,0},{4,0},{-3,-3},{0,-4},{3,-3}};
		for (int [] pair:notableSquares) {
			boolean ooX = pair[0]+targetX > Resources.ARENA_X || pair[0]+targetX < 0;
			boolean ooY = pair[1]+targetY > Resources.ARENA_Y || pair[1]+targetY < 0;
			if (ooX || ooY) {
				continue;
			}
			else {
				double dist  = Math.sqrt((pair[0]+targetX)*(pair[0]+targetX)+(pair[1]+targetY)*(pair[1]+targetY));
				if (dist < shortest_dist) {
					result [0] = pair[0] + targetX;
					result [1] = pair[1] + targetY;
					shortest_dist = dist;
				}
			}
		}
		return result;
	}
	
	/**
	 * class for waypoints on the map used for A* algorithm
	 * @author yp
	 */
	private class Waypoint {
		int X;
		int Y;

		Waypoint (int x, int y){
			this.X = x;
			this.Y = y;
		}
		
		/**
		 * returns the distance needed to travel by cardinal directions from one waypoint to another
		 * @param anotherone another way point
		 * @return
		 */
		int getDistanceTo(Waypoint anotherone) {
			return Math.abs(this.X - anotherone.X) + Math.abs(this.Y - anotherone.Y);
		}
		
		boolean equals (Waypoint anotherone) {
			return this.X == anotherone.X && this.Y == anotherone.Y;
		}
		
		Waypoint [] getAdjacentSquares() {
			return null;
			
		}
		
	}
	
	 
	
}
