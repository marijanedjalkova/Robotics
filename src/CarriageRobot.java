import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

import geometry.IntPoint;
import renderables.Renderable;

public class CarriageRobot extends PotentialFieldsRobot {

	private final double MAXSPEED = 10;
	private final double MINSPEED = 2;
	double width; //distance between the centre of 2 wheels

	public CarriageRobot(String imagePath, IntPoint startingLocation, IntPoint goalLocation, int radius,
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles) {
		super(imagePath, startingLocation, goalLocation, radius, sensorRange, sensorDensity, goalRadius, obstacles);
		width = 0.5;
	}
	
	@Override
	public boolean move(){
		IntPoint moveTo = evaluateSamplePoints(); //Pick a sample point to move towards
		if (moveTo == null) return false;
		PotentialPoint makeMove = evaluateMovePoint(moveTo); //Find the best move point using current sample as goal
		if (makeMove == null) return false;
		moveTowardsPoint(makeMove); //Make the move
		return true;
	}

	
	public void moveTowardsPoint(PotentialPoint p){
		coords.x += (int)(p.changeX);		
		coords.y += (int)(p.changeY);
		heading = p.heading;
	}
	
	
	public PotentialPoint evaluateMovePoint(IntPoint moveTo){
		double step = 0.5;
		ArrayList<PotentialPoint> points = new ArrayList<PotentialPoint>();
		
		for (double l = MINSPEED; l < MAXSPEED; l = l + step){
			for (double r = MINSPEED; r < MAXSPEED; r = r + step){
				if (Math.abs(r - l) <= 6){
					PotentialPoint potential = getPointTowards(r, l);
					if (potential != null){
						Line2D.Double line = new Line2D.Double(coords.x, coords.y, 
								coords.x + potential.changeX, coords.y + potential.changeY);
						if (intersects(line) == null)
							points.add(potential);
					}
				}

			}
		}
		if (points.size() == 0)
			return null;
		double[]moveValues = new double[points.size()];
		for (int i = 0; i < moveValues.length; i++){
			double newX =(coords.x + points.get(i).changeX);
			double newY =(coords.y + points.get(i).changeY);
			moveValues[i] = evalMoveDouble(newX, newY, moveTo) * 
					doubleDistance(newX, newY, moveTo.x, moveTo.y);
		}

		return points.get(minIndex(moveValues));
	}
	
	public double evalMoveDouble(double x, double y, IntPoint miniGoal){
		//Get distances to goal & all visible objects
				double goalDist = (doubleDistance(x, y, miniGoal.x, miniGoal.y)-radius) / 10; //Everything is divided by 10 because otherwise the numbers get too big
				double[] obsDists = new double[visibleObstacles.size()];
				for(int i=0;i<visibleObstacles.size();i++) {
					//Distance is set to 0 if it's closer than the radius to the obstacle
					obsDists[i] = (doubleDistance(x, y, 
							visibleObstacles.get(i).x, visibleObstacles.get(i).y) - radius) 
							<= 0 ? 0 : (doubleDistance(x, y, visibleObstacles.get(i).x,
									visibleObstacles.get(i).y) - radius) / 10;
				}
				//Calculate field power - x^2 so value gets small as distance decreases
				double goalField = Math.pow(goalDist, 2);
				//obs. field power is sum of all obstacles, and gets v. large as distance decreases and vice versa
				double obsField = 0;
				for(int i=0;i<visibleObstacles.size();i++) {
					if(obsDists[i] <= 0) {
						obsField = Double.MAX_VALUE;
						break;
					} else if (obsDists[i] > sensorRange) {
						continue;
					}
					obsField += Math.pow(Math.E, -1 / ((sensorRange) - obsDists[i])) / (obsDists[i]);
				}
				return 10*goalField + Math.pow(2*radius,2)*4750*obsField / (sensorDensity*sensorRange);
	}
	
	public double doubleDistance(double x1, double y1, double x2, double y2){
		return Math.sqrt(Math.pow((x1-x2), 2) + Math.pow((y1-y2), 2));
	}
	
	public PotentialPoint getPointTowards(double r, double l){
		double R = width/2 * ((r + l)/(r - l));
		//System.out.println("R " + R);
		double w = (r - l)/width;
		double wRad = mod(Math.toRadians(w), 2*Math.PI);
		if (Math.floor(w*100)==0 || Math.ceil(w*100)==0) {
			return null;
		}
		
		double changeX = Math.sin(heading)*R*Math.cos(wRad)+Math.sin(wRad)*R*Math.cos(heading) - R*Math.sin(heading);
		double changeY = Math.sin(heading)*R*Math.sin(wRad)-Math.cos(wRad)*R*Math.cos(heading) + R*Math.cos(heading);
		
		while (Math.abs(changeX) < 1  && Math.abs(changeY) < 1) {	
			changeX = changeX * 10;			
			changeY = changeY * 10;		
		}
		double newHeading = mod(heading + wRad, 2*Math.PI);
		return new PotentialPoint(newHeading, changeX, changeY);
	}
	
	
}
