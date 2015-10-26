import java.util.ArrayList;
import java.util.List;

import geometry.IntPoint;
import renderables.Renderable;

public class CarriageRobot extends PotentialFieldsRobot {
	double Vr, Vl; //speeds of wheels
	private final double MAXSPEED = 10;
	private final double MINSPEED = 0.01;
	double l; //distance between the centre of 2 wheels

	public CarriageRobot(String imagePath, IntPoint startingLocation, IntPoint goalLocation, int radius,
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles) {
		super(imagePath, startingLocation, goalLocation, radius, sensorRange, sensorDensity, goalRadius, obstacles);
		l = 1;
		Vr = 5;
		Vl = 5;
	}
	
	@Override
	public boolean move(){
		PotentialPoint makeMove = evaluateMovePoints(); //Find the best move point using current sample as goal
		if (makeMove == null) return false;
		moveTowards(makeMove); //Make the move
		return true;
	}
	
	public void moveTowards(PotentialPoint p){
		System.out.println(" changeX " + p.changeX + " changeY " + p.changeY);
		coords.x += (int)(p.changeX);		
		coords.y += (int)(p.changeY);
		System.out.println("newX " + coords.x + " newY " + coords.y);
		heading = p.heading;
	}
	
	public PotentialPoint evaluateMovePoints(){
		double step = 1;
		ArrayList<PotentialPoint> points = new ArrayList<PotentialPoint>();
		double maxChange = 5;
		double lLow = Vl - maxChange < MINSPEED ? MINSPEED : Vl - maxChange;
		double lHigh = Vl + maxChange > MAXSPEED ? MAXSPEED : Vl + maxChange;
		double rLow = Vr - maxChange < MINSPEED ? MINSPEED : Vr - maxChange;
		double rHigh = Vr + maxChange > MAXSPEED ? MAXSPEED : Vr + maxChange;
		System.out.println("====vl " + Vl + " vr " + Vr);
		System.out.println("llow " + lLow + " lHigh " + lHigh);
		System.out.println("rlow " + rLow + " rHigh " + rHigh);
		
		for (double l = lLow; l < lHigh; l = l + step){
			for (double r = rLow; r < rHigh; r = r + step){
				PotentialPoint potential = getPointTowards(r, l);
				points.add(potential);
			}
		}
		if (points.size() == 0)
			return null;
		double[]moveValues = new double[points.size()];
		for (int i = 0; i < moveValues.length; i++){
			int newX =(int) (coords.x + points.get(i).changeX);
			int newY =(int) (coords.y + points.get(i).changeY);
			IntPoint p = new IntPoint(newX, newY);
			moveValues[i] = evalMove(p, goal);
		}
		return points.get(minIndex(moveValues));
	}
	
	public PotentialPoint getPointTowards(double r, double l){
		double R = l/2 * ((r + l)/(r - l));
		//System.out.println("R " + R);
		double w = (r - l)/l;
		double wRad = mod(Math.toRadians(w), 2*Math.PI);
		//System.out.println("wRad " + wRad);
		double ICCx = coords.x - R*Math.sin(heading);
		double ICCy = coords.y + R*Math.cos(heading);
		double newX = Math.cos(wRad)*(coords.x - ICCx) - Math.sin(wRad)*(coords.y - ICCy) + ICCx;
		double newY = Math.sin(wRad)*(coords.x - ICCx) + Math.cos(wRad)*(coords.y - ICCy) + ICCy;
		
		
		double changeX = R*(Math.sin(heading)*Math.cos(wRad)+Math.sin(wRad)*Math.cos(heading) - Math.sin(heading));
		double changeY = R*(Math.sin(heading)*Math.sin(wRad)-Math.cos(wRad)*Math.cos(heading) + Math.cos(heading));
		
		while (Math.abs(changeX) < 1  && Math.abs(changeY) < 1) {	
			changeX*=10;			
			changeY*=10;		
		}

		//System.out.println(" changeX " + changeX + " changeY " + changeY);
				
		double newHeading = mod(heading + wRad, 2*Math.PI);
		//System.out.println("heading " + newHeading);
		return new PotentialPoint(newX, newY, newHeading, changeX, changeY);
	}
	
	
}
