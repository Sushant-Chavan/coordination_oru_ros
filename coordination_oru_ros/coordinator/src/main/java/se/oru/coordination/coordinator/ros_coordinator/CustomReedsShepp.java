package se.oru.coordination.coordinator.ros_coordinator;

import java.util.ArrayList;
import java.io.File;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;

import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.GeometrySmoother;
import se.oru.coordination.coordination_oru.util.GeometrySmoother.SmootherControl;
import se.oru.coordination.coordinator.ros_coordinator.ReedsSheppCarPlannerLib.PathPose;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeTrackerROS.VEHICLE_STATE;

public class CustomReedsShepp extends AbstractMotionPlanner {
	
	private TrajectoryEnvelopeCoordinatorROS tec = null;
    private int robotID = -1;
    
    private double robotRadius = 1.0;
	private PointerByReference path = null;
	private IntByReference pathLength = null;
	private double distanceBetweenPathPoints = 0.5;
	private double turningRadius = 1.0;
	private Coordinate[] collisionCircleCenters = null;
	
	public static ReedsSheppCarPlannerLib INSTANCE = null;
	static {
		NativeLibrary.addSearchPath("simplereedssheppcarplanner", "SimpleReedsSheppCarPlanner");
		INSTANCE = Native.loadLibrary("simplereedssheppcarplanner", ReedsSheppCarPlannerLib.class);
    }

    @Override
	public void setFootprint(Coordinate ... coords) {
		super.setFootprint(coords);
		GeometryFactory gf = new GeometryFactory();
		Coordinate[] newCoords = new Coordinate[coords.length+1];
		for (int i = 0; i < coords.length; i++) {
			newCoords[i] = coords[i];
		}
		newCoords[newCoords.length-1] = coords[0];
		Polygon footprint = gf.createPolygon(newCoords);
		GeometrySmoother gs = new GeometrySmoother(gf);
		SmootherControl sc = new SmootherControl() {
	        public double getMinLength() {
	            return robotRadius;
	        }
	        
	        public int getNumVertices(double length) {
	            return (int)(length/(2*robotRadius))+2;
	        }
	    };
	    gs.setControl(sc);
	    Polygon smoothFootprint = gs.smooth(footprint, 1.0);
		collisionCircleCenters = smoothFootprint.getCoordinates();
	}
	
	public Coordinate[] getCollisionCircleCenters() {
		return collisionCircleCenters;
	}

	public void setCirclePositions(Coordinate ... circlePositions) {
		this.collisionCircleCenters = circlePositions;
	}

	@Deprecated
	public void setRobotRadius(double rad) {
		this.robotRadius = rad;
	}

	public void setRadius(double rad) {
		this.robotRadius = rad;
	}

	public void setDistanceBetweenPathPoints(double maxDistance) {
		this.distanceBetweenPathPoints = maxDistance;
	}

	public void setTurningRadius(double rad) {
		this.turningRadius = rad;
    }
    
    public String getOriginalFilename() {
        String[] bits = this.mapFilenameBAK.split("/");
        String experienceDBNameWithExtension = bits[bits.length-1];
        String[] extensionBits = experienceDBNameWithExtension.split("\\.");
        return extensionBits[0];
    }

    public enum PLANNER_TYPE{
        SIMPLE,
        LIGHTNING,
        THUNDER
    }

	public CustomReedsShepp(int robotID, TrajectoryEnvelopeCoordinatorROS tec) {
		deleteDir(new File(TEMP_MAP_DIR));
        new File(TEMP_MAP_DIR).mkdir();

		this.tec = tec;
		this.robotID = robotID;
	}
	
	@Override
	public boolean doPlanning() {
		//TODO: refuse to do planning if the robot is not idle,
		//		as compute_task service will use current pose of
		//		robot as initial pose. 
		if (!(
				tec.getVehicleState(robotID).equals(VEHICLE_STATE.AT_CRITICAL_POINT) ||
				tec.getVehicleState(robotID).equals(VEHICLE_STATE.WAITING_FOR_TASK) ||
				tec.getVehicleState(robotID).equals(VEHICLE_STATE._IGNORE_))) {
			System.out.println("Not planning because Robot" + robotID + " is not idle (in state " + tec.getVehicleState(robotID) + ")");
			return false;
		}
		
        ArrayList<PoseSteering> finalPath = new ArrayList<PoseSteering>();  
        PLANNER_TYPE plannerType = PLANNER_TYPE.SIMPLE;
		for (int i = 0; i < this.goal.length; i++) {
			Pose start_ = null;
            Pose goal_ = this.goal[i];
            start_ = tec.getRobotReport(robotID).getPose();
			path = new PointerByReference();
			pathLength = new IntByReference();
			if (collisionCircleCenters == null) {
				if (!INSTANCE.plan(mapFilename, mapResolution, robotRadius, start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(), goal_.getY(), goal_.getTheta(), path, pathLength, distanceBetweenPathPoints, turningRadius)) return false;
			}
			else {
				double[] xCoords = new double[collisionCircleCenters.length];
				double[] yCoords = new double[collisionCircleCenters.length];
				int numCoords = collisionCircleCenters.length;
				for (int j = 0; j < collisionCircleCenters.length; j++) {
					xCoords[j] = collisionCircleCenters[j].x;
					yCoords[j] = collisionCircleCenters[j].y;
				}
				metaCSPLogger.info("Path planning with " + collisionCircleCenters.length + " circle positions");
				if (this.mapFilename != null) {
                    String experienceDBName = this.getOriginalFilename();
					if (!INSTANCE.plan_multiple_circles(mapFilename, mapResolution, robotRadius, xCoords, yCoords, numCoords, start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(), goal_.getY(), goal_.getTheta(), path, pathLength, distanceBetweenPathPoints, turningRadius, plannerType.ordinal(), experienceDBName)) return false;					
				}
				else {
                    if (start_ == null)
                    {
                        System.out.println(">>>>>>>>>>>>>>>>>>>> Start is NULL");
                    }
                    if (goal_ == null)
                    {
                        System.out.println(">>>>>>>>>>>>>>>>>>>> Goal is NULL");
                    }
					if (!INSTANCE.plan_multiple_circles_nomap(xCoords, yCoords, numCoords, start_.getX(), start_.getY(), start_.getTheta(), goal_.getX(), goal_.getY(), goal_.getTheta(), path, pathLength, distanceBetweenPathPoints, turningRadius, plannerType.ordinal())) return false;					
				}
			}
			final Pointer pathVals = path.getValue();
			final PathPose valsRef = new PathPose(pathVals);
			valsRef.read();
			int numVals = pathLength.getValue();
			if (numVals == 0) return false;
			PathPose[] pathPoses = (PathPose[])valsRef.toArray(numVals);
			if (i == 0) finalPath.add(new PoseSteering(pathPoses[0].x, pathPoses[0].y, pathPoses[0].theta, 0.0));
			for (int j = 1; j < pathPoses.length; j++) finalPath.add(new PoseSteering(pathPoses[j].x, pathPoses[j].y, pathPoses[j].theta, 0.0));
			INSTANCE.cleanupPath(pathVals);
		}
		this.pathPS = finalPath.toArray(new PoseSteering[finalPath.size()]);
		return true;
	}

}
