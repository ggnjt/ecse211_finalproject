package ca.mcgill.ecse211.finalproject.phase2;

import static ca.mcgill.ecse211.finalproject.Resources.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;
import java.util.Stack;
import ca.mcgill.ecse211.finalproject.Navigation;

/**
 * An implementation of the A* path finding algorithm which only allows cardinal movements on the map
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
  static private Square[][] map;

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
  private static int targetX;
  /**
   * target Y coordinate
   */
  private static int targetY;

  /**
   * constructor which initialized the map and the path finder. the enemy tunnel and base are not considered and
   * regarded as the same as the river this should be called after localization
   * 
   * @param arenaSizeX arena x size -1
   * @param arenaSizeY arena y size -1
   * @param tunnelLLX lower left corner X coordinate of OUR tunnel
   * @param tunnelLLY lower left corner Y coordinate of OUR tunnel
   * @param tunnelURX upper right corner X coordinate of OUR tunnel
   * @param tunnelURY upper right corner Y coordinate of OUR tunnel
   * @param baseLLX lower left corner X coordinate of OUR base
   * @param baseLLY lower left corner Y coordinate of OUR base
   * @param baseURX upper right corner X coordinate of OUR base
   * @param baseURY upper right corner Y coordinate of OUR base
   * @param islandLLX lower left corner X coordinate of the central island
   * @param islandLLY lower left corner Y coordinate of the central island
   * @param islandURX upper right corner X coordinate of the central island
   * @param islandURY upper right corner Y coordinate of the central island
   */
  public PathFinder(boolean isRedTeam) {
  	int corner = isRedTeam ? redCorner : greenCorner;
  	Region base = isRedTeam ? red : green;
  	Region tn = isRedTeam ? tnr : tng;
  	
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
    // set tunnel
    for (int i = (int) tn.ll.x; i < (int) tn.ur.x; i++) {
      for (int j = (int) tn.ll.y; j < (int) tn.ur.y; j++) {
        map[i][j].setStatus(2);
      }
    }
    // set island
    for (int i = (int) island.ll.x; i < (int) island.ur.x; i++) {
      for (int j = (int) island.ll.y; j < (int) island.ur.y; j++) {
        map[i][j].setStatus(3);
      }
    }

    switch (corner) {
      case 0:
        currentX = 0;
        currentY = 0;
        navigation.orientation = Navigation.Orientation.NORTH;
        break;
      case 1:
        currentX = ARENA_X - 1;
        currentY = 0;
        navigation.orientation = Navigation.Orientation.WEST;
        break;
      case 2:
        currentX = ARENA_X - 1;
        currentY = ARENA_Y - 1;
        navigation.orientation = Navigation.Orientation.SOUTH;
        break;
      case 3:
        currentX = 0;
        currentY = ARENA_Y - 1;
        navigation.orientation = Navigation.Orientation.EAST;
        break;
    }
    navigation.xTile = currentX;
    navigation.yTile = currentY;
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
   * this checks the heuristic cost of an adjacent square, updates the final cost and adds it into the priority queue
   * 
   * @param current current square probing from
   * @param target square being probed
   * @param cost cost increment of the target
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
   * runs the A* algorithm and finds the shortest path to target using only cardinal movements
   * 
   * @return a list of int[] of size 2, which represents the list of squares the robot should move to in order to reach
   *         to the target
   */
  public ArrayList<int[]> findPath() {

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
        // at this point, you are at a target and the route can be traced back by using
        // the parent Squares
        ArrayList<int[]> result = new ArrayList<int[]>();
        // we use a Stack to help reverse the linked list and calculate the movement
        // (instead of using squares)
        Stack<Square> ghettoStack = new Stack<Square>();
        ghettoStack.push(current);
        while (current.parent != null) {
          ghettoStack.push(current.parent);
          current = current.parent;
        }
        while (!ghettoStack.isEmpty()) {
          current = ghettoStack.pop();
          int[] entry = {current.X, current.Y};
          result.add(entry);
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

  /**
   * ***** code copied over from lab 5 **** finds the square which the robot can launch from 4 squares away from the
   * target
   * 
   * @param targetX x-coordinate of the target of the ball
   * @param targetY y-coordinate of the target of the ball
   * @return an int array of size 2, which represents the coordinates of the ideal launch point
   */
  private static int[] findLaunchPointToTarget(int targetX, int targetY) {

    int[] result = new int[2];
    double shortest_dist = 100;
    int[][] notableSquares = {{-3, 3}, {0, 4}, {3, 3}, {-4, 0}, {4, 0}, {-3, -3}, {0, -4}, {3, -3}};
    for (int[] pair : notableSquares) {
      boolean ooX = pair[0] + targetX > ARENA_X || pair[0] + targetX < 0;
      boolean ooY = pair[1] + targetY > ARENA_Y || pair[1] + targetY < 0;
      if (ooX || ooY) {
        continue;
      } else {
        double dist = Math.sqrt((pair[0] + targetX) * (pair[0] + targetX) + (pair[1] + targetY) * (pair[1] + targetY));
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
    /**
     * heuristic cost for the square to get to target, calculated by the Euclidean distance
     */
    double hCost = 0;
    /**
     * final cost for the target square to get to the target. sum of hCost and the number of square traveled
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
     * number represents the type of terrain of the sqaure 0 => river or enemy base/tunnel 3 => central island 2 =>
     * tunnel 1 => base -2=> obstacle
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
      return Math
          .sqrt((this.X - anotherone.X) * (this.X - anotherone.X) + (this.Y - anotherone.Y) * (this.Y - anotherone.Y));
    }

    /**
     * sets the values for the status of the square, each number represents a type of terrain 0 => river or enemy
     * base/tunnel 3 => central island 2 => tunnel 1 => base -2=> obstacle
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
