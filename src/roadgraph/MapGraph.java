/**
 * @author UCSD MOOC development team and YOU
 * @author Belnarto
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * @author Belnarto
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private HashMap< GeographicPoint,HashMap<GeographicPoint,Edge> > map;
	private List<GeographicPoint> path;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		map = new HashMap<>();
		path = new LinkedList<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return map.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		//Set<GeographicPoint> setToReturn = new HashSet<GeographicPoint>();
		//for (GeographicPoint gp : )
		return map.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		int numToReturn = 0;
		for (GeographicPoint gp : getVertices()) {
			numToReturn += map.get(gp).size();
		}
		return numToReturn/2;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if (location != null && !map.containsKey(location)) {
			map.put(location, new HashMap<>());
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if (from == null || to == null || roadName == null || roadType == null || 
				length <= 0.0 || !map.containsKey(from) || !map.containsKey(to)) {
			throw new IllegalArgumentException();
		} else {
			Edge edgeToAdd = new Edge(from,to,roadName,roadType,length);
			map.get(from).put(to, edgeToAdd);
		}
		//TODO: Implement this method in WEEK 3
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	@SuppressWarnings("unchecked")
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		HashSet<GeographicPoint> visited = new HashSet<>();
		Deque<GeographicPoint> bufferPath = new LinkedList<>();
		Deque<Deque<GeographicPoint>> queueNexts = new LinkedList<>();
		GeographicPoint bufferGP = null;
		
		if (start == null || goal == null || nodeSearched == null) {
			return null;
		}
		
		bufferPath.addFirst(start);
		queueNexts.addLast(bufferPath);
		
		
		while (!queueNexts.isEmpty() && !visited.contains(goal)) {
			bufferPath = queueNexts.pollFirst();
			bufferGP = bufferPath.getLast();
			if (!visited.contains(bufferGP)) {
				visited.add(bufferGP);
				nodeSearched.accept(bufferGP);
				Set<GeographicPoint> neighbors = map.get(bufferGP).keySet();
				for (GeographicPoint neighbor : neighbors) {
					if (!visited.contains(neighbor)) {
						queueNexts.add(new LinkedList<>(bufferPath));
						queueNexts.peekLast().addLast(neighbor);

					}
				}
			}
		}
		
		if (bufferPath instanceof LinkedList<?>) {
			return (List<GeographicPoint>) bufferPath;
		} else {
			return null;
		}
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		List<GeographicPoint> visited = new LinkedList<>();
		Route bufferRoute = new Route();
		Queue<Route> queueNexts = new PriorityQueue<>();
		GeographicPoint bufferGP = null;
		
		bufferRoute.addFirst(start);
		queueNexts.add(bufferRoute);	
		
		while (!queueNexts.isEmpty() && !queueNexts.peek().getLast().equals(goal)) {
			bufferRoute = queueNexts.poll();
			bufferGP = bufferRoute.getLast();
			if (!visited.contains(bufferGP)) {
				visited.add(bufferGP);
				nodeSearched.accept(bufferGP);
				Set<GeographicPoint> neighbors = map.get(bufferGP).keySet();
				for (GeographicPoint neighbor : neighbors) {
					if (!visited.contains(neighbor)) {
						Route newRouteToAdd = new Route(bufferRoute);
						newRouteToAdd.addActualDistance(map.get(bufferGP).get(neighbor).getLength());
						newRouteToAdd.addLast(neighbor);
						queueNexts.add(newRouteToAdd);
					}
				}
			}
		}
		
		visited.add(goal);
		nodeSearched.accept(goal);
		
		System.out.println("Visited vertices number: " + visited.size());
		System.out.println("Path of visitting vertices: " + visited);
		return queueNexts.poll().getPath();
	}	
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		List<GeographicPoint> visited = new LinkedList<>();
		Route bufferRoute = new Route();
		Queue<Route> queueNexts = new PriorityQueue<>();
		GeographicPoint bufferGP = null;
		
		bufferRoute.addFirst(start);
		queueNexts.add(bufferRoute);	
		
		while (!queueNexts.isEmpty() && !queueNexts.peek().getLast().equals(goal)) {
			bufferRoute = queueNexts.poll();
			bufferGP = bufferRoute.getLast();
			if (!visited.contains(bufferGP)) {
				visited.add(bufferGP);
				//System.out.println(bufferGP);
				nodeSearched.accept(bufferGP);
				Set<GeographicPoint> neighbors = map.get(bufferGP).keySet();
				for (GeographicPoint neighbor : neighbors) {
					if (!visited.contains(neighbor)) {
						Route newRouteToAdd = new Route(bufferRoute);
						newRouteToAdd.addActualDistance(map.get(bufferGP).get(neighbor).getLength());
						newRouteToAdd.addPredictedDistance(neighbor.distance(goal));
						newRouteToAdd.addLast(neighbor);
						queueNexts.add(newRouteToAdd);
					}
				}
			}
		}
		
		
		visited.add(goal);
		nodeSearched.accept(goal);
		
		System.out.println("\nVisited vertices number: " + visited.size());
		System.out.println("Path of visitting vertices: " + visited);
		
		return queueNexts.poll().getPath();
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		System.out.println(firstMap.getNumVertices());
		System.out.println(firstMap.getVertices());
		System.out.println(firstMap.getNumEdges());
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint goal = new GeographicPoint(8.0, -1.0);
		System.out.println("bfs: " + firstMap.bfs(start, goal));
		
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("\nTest 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		System.out.println("Size of path: " + testroute.size());
		System.out.println("Path: " + testroute);

		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println("Size of path: " + testroute2.size());
		System.out.println("Path: " + testroute2);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("\nTest 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		//System.out.println(testroute.size());
		//System.out.println(testroute2.size());
		
		
			
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("\nTest 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		//System.out.println(testroute.size());
		//System.out.println(testroute2.size());
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}

class Edge {
	private GeographicPoint start;
	private GeographicPoint end;
	private String streetName;
	private String streetType;
	private double length;
	
	public Edge (GeographicPoint start, GeographicPoint end, String streetName, String streetType, double length) {
		this.start=start;
		this.end=end;
		this.streetName=streetName;
		this.streetType=streetType;
		this.length=length;
	}
	
	public double getLength() {
		return length;
	}
}

class Route implements Comparable<Route> {
	private double actualDistance;
	private double predictedDistance;
	private double totalDistance;
	private Deque<GeographicPoint> path;
	
	public Route () {
		actualDistance = 0;
		predictedDistance = 0;
		totalDistance = actualDistance + predictedDistance;
		path = new LinkedList<>();
	}

	public Route (Route o) {
		actualDistance = o.actualDistance;
		predictedDistance = o.predictedDistance;
		totalDistance = actualDistance + predictedDistance;
		path = new LinkedList<>(o.path);
	}
	
	public double getActualDistance() {
		return actualDistance;
	}
	
	public double getPredictedDistance() {
		return predictedDistance;
	}
	
	public double getTotalDistance() {
		return totalDistance;
	}	
	
	public void addActualDistance(double actualDistance) {
		this.actualDistance += actualDistance;
		totalDistance += actualDistance;
	}

	
	public void addPredictedDistance(double predictedDistance) {
		totalDistance -= this.predictedDistance;
		this.predictedDistance = predictedDistance;
		totalDistance += predictedDistance;
	}
	
	public void addFirst(GeographicPoint georgPoint) {
		if (georgPoint == null) {
			throw new IllegalArgumentException();
		} else {
			path.addFirst(georgPoint);
		}
	}

	public void addLast(GeographicPoint georgPoint) {
		if (georgPoint == null) {
			throw new IllegalArgumentException();
		} else {
			path.addLast(georgPoint);
		}
	}
	
	public GeographicPoint getLast() {
			return (GeographicPoint) path.getLast().clone();
	}
	
	@SuppressWarnings("unchecked")
	public List<GeographicPoint> getPath() {
		if (path instanceof LinkedList<?>) {
			return (List<GeographicPoint>) path;
		} else {
			return null;
		}
	}
	
	@Override
	public int compareTo(Route o) {
		if (o == null) {
			throw new IllegalArgumentException();
		} else if (this.totalDistance > o.totalDistance) {
			return 1;
		} else if (this.totalDistance == o.totalDistance) {
			return 0;
		} else {
			return -1;
		}
		
	}
}

