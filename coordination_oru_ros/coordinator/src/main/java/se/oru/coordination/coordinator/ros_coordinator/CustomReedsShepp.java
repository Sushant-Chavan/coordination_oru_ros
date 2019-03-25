package se.oru.coordination.coordinator.ros_coordinator;

import java.util.ArrayList;
import java.util.HashMap;
import java.io.File;

import com.sun.jna.Native;
import com.sun.jna.NativeLibrary;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.PointerByReference;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Polygon;

import geometry_msgs.Point;
import orunav_msgs.ComputeTask;
import orunav_msgs.ComputeTaskRequest;
import orunav_msgs.ComputeTaskResponse;
import orunav_msgs.RobotTarget;
import orunav_msgs.Shape;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.GeometrySmoother;
import se.oru.coordination.coordination_oru.util.GeometrySmoother.SmootherControl;
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
		
		// this.callComputeTaskService(this.goal[0], robotID);
		// while (computing) try { Thread.sleep(100); } catch (InterruptedException e) { e.printStackTrace(); }
        // return outcome;
        return false;
	}

}
