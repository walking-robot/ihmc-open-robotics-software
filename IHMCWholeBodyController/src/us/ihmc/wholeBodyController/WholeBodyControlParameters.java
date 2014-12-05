package us.ihmc.wholeBodyController;
 
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public interface WholeBodyControlParameters
{
	public CapturePointPlannerParameters getCapturePointPlannerParameters();

	public ArmControllerParameters getArmControllerParameters();

	public WalkingControllerParameters getWalkingControllerParameters();
	
	public WalkingControllerParameters getMultiContactControllerParameters();
	
	public DRCRobotContactPointParameters getContactPointParameters();
	
	public double getControllerDT();
	
	public SDFFullRobotModel createFullRobotModel();

	public SDFRobot createSdfRobot(boolean createCollisionMeshes);
}
