import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

import geometry.IntPoint;
import renderables.Renderable;

public class CarriageRobot extends PotentialFieldsRobot {

	private final double MAXSPEED = 10;
	private final double MINSPEED = 2;
	double width; // distance between the centre of 2 wheels
	public PotentialPoint nextMove;
	public boolean quadratic, arc, euclidean, manhattan;

	public CarriageRobot(String imagePath, IntPoint startingLocation, IntPoint goalLocation, int radius,
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles) {
		super(imagePath, startingLocation, goalLocation, radius, sensorRange, sensorDensity, goalRadius, obstacles);
		width = 0.5;
		nextMove = null;
	}

	@Override
	public boolean move() {
		IntPoint moveTo = evaluateSamplePoints(); // Pick a sample point to move
													// towards
		if (moveTo == null)
			return false;
		PotentialPoint makeMove = evaluateMovePoint(moveTo); // Find the best
																// move point
																// using current
																// sample as
																// goal
		if (makeMove == null)
			return false;
		nextMove = makeMove;
		moveTowardsPoint(makeMove); // Make the move
		return true;
	}

	public PotentialPoint getNextMove() {
		return nextMove;
	}

	public void moveTowardsPoint(PotentialPoint p) {
		coords.x += (int) (p.changeX);
		coords.y += (int) (p.changeY);
		heading = p.heading;
	}

	public PotentialPoint evaluateMovePoint(IntPoint moveTo) {
		double step = 0.5;
		ArrayList<PotentialPoint> points = new ArrayList<PotentialPoint>();

		for (double l = MINSPEED; l < MAXSPEED; l = l + step) {
			for (double r = MINSPEED; r < MAXSPEED; r = r + step) {
				if (Math.abs(r - l) <= 6) {
					PotentialPoint potential = getPointTowards(r, l);
					if (potential != null) {
						Line2D.Double line = new Line2D.Double(coords.x, coords.y, coords.x + potential.changeX,
								coords.y + potential.changeY);
						if (intersects(line) == null)
							points.add(potential);
					}
				}

			}
		}
		if (points.size() == 0)
			return null;
		double[] moveValues = new double[points.size()];
		for (int i = 0; i < moveValues.length; i++) {
			double newX = (coords.x + points.get(i).changeX);
			double newY = (coords.y + points.get(i).changeY);
			moveValues[i] = evalMoveDouble(newX, newY, moveTo);
		}

		return points.get(minIndex(moveValues));
	}

	public double evalMoveDouble(double x, double y, IntPoint miniGoal) {
		// Get distances to goal & all visible objects
		double goalDist = 0;
		if (quadratic)
			goalDist = (quadraticDistance(x, y, miniGoal.x, miniGoal.y) - radius) / 10;
		else if (arc)
			goalDist = (arcDistance(coords.x, coords.y, x, y, goal.x, goal.y) - radius) / 10;
		else if (manhattan)
			goalDist = (manhattanDistance(x, y, miniGoal.x, miniGoal.y) - radius) / 10;
		else if (euclidean)
			goalDist = (euclideanDistance(x, y, miniGoal.x, miniGoal.y)-radius) / 10;
		double[] obsDists = new double[visibleObstacles.size()];
		for (int i = 0; i < visibleObstacles.size(); i++) {
			// Distance is set to 0 if it's closer than the radius to the
			// obstacle
			obsDists[i] = (euclideanDistance(x, y, visibleObstacles.get(i).x, visibleObstacles.get(i).y) - radius) <= 0
					? 0 : (euclideanDistance(x, y, visibleObstacles.get(i).x, visibleObstacles.get(i).y) - radius) / 10;
		}
		// Calculate field power - x^2 so value gets small as distance decreases
		double goalField = Math.pow(goalDist, 2);
		// obs. field power is sum of all obstacles, and gets v. large as
		// distance decreases and vice versa
		double obsField = 0;
		for (int i = 0; i < visibleObstacles.size(); i++) {
			if (obsDists[i] <= 0) {
				obsField = Double.MAX_VALUE;
				break;
			} else if (obsDists[i] > sensorRange) {
				continue;
			}
			obsField += Math.pow(Math.E, -1 / ((sensorRange) - obsDists[i])) / (obsDists[i]);
		}
		return 10 * goalField + Math.pow(2 * radius, 2) * 4750 * obsField / (sensorDensity * sensorRange);
	}

	public double arcDistance(double curX, double curY, double planX, double planY, double goalX, double goalY) {

		double yDelta_a = planY - curY;
		double xDelta_a = planX - curX;
		double yDelta_b = goalY - planY;
		double xDelta_b = goalX - planX;

		double aSlope = yDelta_a / xDelta_a;
		double bSlope = yDelta_b / xDelta_b;
		double centreX = (aSlope * bSlope * (curY - goalY) + bSlope * (curX + planX) - aSlope * (planX + goalX))
				/ (2 * (bSlope - aSlope));
		double centreY = -1 * (centreX - (curX + planX) / 2) / aSlope + (curY + planY) / 2;

		double diameter = 2 * euclideanDistance(centreX, centreY, curX, curY);
		Line2D line1 = new Line2D.Double(centreX, centreY, curX, curY);
		Line2D line2 = new Line2D.Double(centreX, centreY, goalX, goalY);
		double angle = angleBetween2Lines(line1, line2);
		return Math.toRadians(angle) * Math.PI * diameter / (Math.toRadians(360));

	}

	public double angleBetween2Lines(Line2D line1, Line2D line2) {
		double angle1 = Math.atan2(line1.getY1() - line1.getY2(), line1.getX1() - line1.getX2());
		double angle2 = Math.atan2(line2.getY1() - line2.getY2(), line2.getX1() - line2.getX2());
		return angle1 - angle2;
	}

	public double manhattanDistance(double x1, double y1, double x2, double y2) {
		return Math.abs(x1 - x2) + Math.abs(y1 - y2);
	}

	public double euclideanDistance(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2));
	}

	public double quadraticDistance(double x1, double y1, double x2, double y2) {
		return Math.pow(euclideanDistance(x1, y1, x2, y2), 2);
	}

	public PotentialPoint getPointTowards(double r, double l) {
		double R = width / 2 * ((r + l) / (r - l));
		// System.out.println("R " + R);
		double w = (r - l) / width;
		double wRad = mod(Math.toRadians(w), 2 * Math.PI);
		if (Math.floor(w * 100) == 0 || Math.ceil(w * 100) == 0) {
			return null;
		}

		double changeX = Math.sin(heading) * R * Math.cos(wRad) + Math.sin(wRad) * R * Math.cos(heading)
				- R * Math.sin(heading);
		double changeY = Math.sin(heading) * R * Math.sin(wRad) - Math.cos(wRad) * R * Math.cos(heading)
				+ R * Math.cos(heading);

		while (Math.abs(changeX) < 1 && Math.abs(changeY) < 1) {
			changeX = changeX * 10;
			changeY = changeY * 10;
		}
		double newHeading = mod(heading + wRad, 2 * Math.PI);
		return new PotentialPoint(newHeading, changeX, changeY);
	}

}
