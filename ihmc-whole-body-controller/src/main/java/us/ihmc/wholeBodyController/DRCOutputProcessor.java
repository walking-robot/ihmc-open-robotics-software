package us.ihmc.wholeBodyController;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface DRCOutputProcessor
{
   public abstract void initialize();

   public abstract void processAfterController(long timestamp);

   public abstract void setLowLevelControllerCoreOutput(FullHumanoidRobotModel controllerRobotModel, JointDesiredOutputList lowLevelControllerCoreOutput);

   public abstract YoVariableRegistry getControllerYoVariableRegistry();
}
