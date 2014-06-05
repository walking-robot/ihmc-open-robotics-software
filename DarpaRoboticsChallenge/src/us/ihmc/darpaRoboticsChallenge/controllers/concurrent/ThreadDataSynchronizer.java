package us.ihmc.darpaRoboticsChallenge.controllers.concurrent;

import java.util.Arrays;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;

public class ThreadDataSynchronizer
{
   private final SDFFullRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder estimatorForceSensorDataHolder;
   private final RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap;

   private final SDFFullRobotModel controllerFullRobotModel;
   private final ForceSensorDataHolder controllerForceSensorDataHolder;
   private final RawJointSensorDataHolderMap controllerRawJointSensorDataHolderMap;

   private final ConcurrentCopier<IntermediateEstimatorStateHolder> estimatorStateCopier;
   
   private long timestamp;
   private long estimatorClockStartTime;
   private long estimatorTick;

   public ThreadDataSynchronizer(DRCRobotModel robotModel)
   {
      estimatorFullRobotModel = robotModel.createFullRobotModel();
      estimatorForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(estimatorFullRobotModel.getForceSensorDefinitions()),
            estimatorFullRobotModel.getRootJoint());
      estimatorRawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(estimatorFullRobotModel);

      controllerFullRobotModel = robotModel.createFullRobotModel();
      controllerForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(controllerFullRobotModel.getForceSensorDefinitions()),
            controllerFullRobotModel.getRootJoint());
      controllerRawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(controllerFullRobotModel);

      IntermediateEstimatorStateHolder.Builder stateCopierBuilder = new IntermediateEstimatorStateHolder.Builder(robotModel,
            estimatorFullRobotModel.getElevator(), controllerFullRobotModel.getElevator(), estimatorForceSensorDataHolder, controllerForceSensorDataHolder,
            estimatorRawJointSensorDataHolderMap, controllerRawJointSensorDataHolderMap);
      estimatorStateCopier = new ConcurrentCopier<IntermediateEstimatorStateHolder>(stateCopierBuilder);

   }

   public boolean receiveControllerState()
   {
      IntermediateEstimatorStateHolder estimatorStateHolder = estimatorStateCopier.getCopyForReading();
      if (estimatorStateHolder != null)
      {
         estimatorStateHolder.getIntoControllerModel();
         timestamp = estimatorStateHolder.getTimestamp();
         estimatorClockStartTime = estimatorStateHolder.getEstimatorClockStartTime();
         estimatorTick = estimatorStateHolder.getEstimatorTick();
         return true;
      }
      else
      {
         return false;
      }
   }

   public void publishEstimatorState(long timestamp, long estimatorTick, long estimatorClockStartTime)
   {
      IntermediateEstimatorStateHolder estimatorStateHolder = estimatorStateCopier.getCopyForWriting();
      estimatorStateHolder.setFromEstimatorModel(timestamp, estimatorTick, estimatorClockStartTime);
      estimatorStateCopier.commit();
   }

   public SDFFullRobotModel getEstimatorFullRobotModel()
   {
      return estimatorFullRobotModel;
   }

   public ForceSensorDataHolder getEstimatorForceSensorDataHolder()
   {
      return estimatorForceSensorDataHolder;
   }

   public SDFFullRobotModel getControllerFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   public ForceSensorDataHolder getControllerForceSensorDataHolder()
   {
      return controllerForceSensorDataHolder;
   }

   public RawJointSensorDataHolderMap getEstimatorRawJointSensorDataHolderMap()
   {
      return estimatorRawJointSensorDataHolderMap;
   }

   public RawJointSensorDataHolderMap getControllerRawJointSensorDataHolderMap()
   {
      return controllerRawJointSensorDataHolderMap;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public long getEstimatorClockStartTime()
   {
      return estimatorClockStartTime;
   }

   public long getEstimatorTick()
   {
      return estimatorTick;
   }

}
