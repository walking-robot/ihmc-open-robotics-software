package us.ihmc.wholeBodyController.concurrent;

import java.util.Arrays;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMapCopier;
import us.ihmc.utilities.GenericCRC32;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateChecksum;
import us.ihmc.utilities.screwTheory.InverseDynamicsJointStateCopier;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.wholeBodyController.WholeBodyControlParameters;
 
public class IntermediateEstimatorStateHolder
{
   private final GenericCRC32 estimatorChecksumCalculator = new GenericCRC32();
   private final GenericCRC32 controllerChecksumCalculator = new GenericCRC32();
   
   private long estimatorTick;
   private long timestamp;
   private long estimatorClockStartTime;
   private long checksum;

   private final InverseDynamicsJointStateChecksum estimatorChecksum; 
   private final InverseDynamicsJointStateChecksum controllerChecksum; 
   
   private final InverseDynamicsJointStateCopier estimatorToIntermediateCopier;
   private final InverseDynamicsJointStateCopier intermediateToControllerCopier;

   private final ForceSensorDataHolder estimatorForceSensorDataHolder;
   private final ForceSensorDataHolder intermediateForceSensorDataHolder;
   private final ForceSensorDataHolder controllerForceSensorDataHolder;
   
   private final RawJointSensorDataHolderMapCopier rawDataEstimatorToIntermadiateCopier;
   private final RawJointSensorDataHolderMapCopier rawDataIntermediateToControllerCopier;

   public IntermediateEstimatorStateHolder(WholeBodyControlParameters wholeBodyControlParameters, RigidBody estimatorRootBody, RigidBody controllerRootBody,
         ForceSensorDataHolder estimatorForceSensorDataHolder, ForceSensorDataHolder controllerForceSensorDataHolder,
         RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap, RawJointSensorDataHolderMap controllerRawJointSensorDataHolderMap)
   {
      SDFFullRobotModel intermediateModel = wholeBodyControlParameters.createFullRobotModel();
      RigidBody intermediateRootBody = intermediateModel.getElevator();

      estimatorChecksum = new InverseDynamicsJointStateChecksum(estimatorRootBody, estimatorChecksumCalculator);
      controllerChecksum = new InverseDynamicsJointStateChecksum(controllerRootBody, controllerChecksumCalculator);
      
      estimatorToIntermediateCopier = new InverseDynamicsJointStateCopier(estimatorRootBody, intermediateRootBody);
      intermediateToControllerCopier = new InverseDynamicsJointStateCopier(intermediateRootBody, controllerRootBody);

      this.estimatorForceSensorDataHolder = estimatorForceSensorDataHolder;
      this.intermediateForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(intermediateModel.getForceSensorDefinitions()));
      this.controllerForceSensorDataHolder = controllerForceSensorDataHolder;
      
      RawJointSensorDataHolderMap intermediateRawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(intermediateModel);
      rawDataEstimatorToIntermadiateCopier = new RawJointSensorDataHolderMapCopier(estimatorRawJointSensorDataHolderMap, intermediateRawJointSensorDataHolderMap);
      rawDataIntermediateToControllerCopier = new RawJointSensorDataHolderMapCopier(intermediateRawJointSensorDataHolderMap, controllerRawJointSensorDataHolderMap);
   }

   public void setFromEstimatorModel(long timestamp, long estimatorTick, long estimatorClockStartTime)
   {
      this.timestamp = timestamp;
      this.estimatorTick = estimatorTick;
      this.estimatorClockStartTime = estimatorClockStartTime;
      
      checksum = calculateEstimatorChecksum();
      estimatorToIntermediateCopier.copy();
      rawDataEstimatorToIntermadiateCopier.copy();
      intermediateForceSensorDataHolder.set(estimatorForceSensorDataHolder);
   }

   public void getIntoControllerModel()
   {
      intermediateToControllerCopier.copy();
      rawDataIntermediateToControllerCopier.copy();
      controllerForceSensorDataHolder.set(intermediateForceSensorDataHolder);
   }
   
   public long getTimestamp()
   {
      return timestamp;
   }
   
   public long getEstimatorTick()
   {
      return estimatorTick;
   }
   
   public long getEstimatorClockStartTime()
   {
      return estimatorClockStartTime;
   }
   
   private long calculateEstimatorChecksum()
   {
      estimatorChecksumCalculator.reset();
      estimatorChecksum.calculate();
      estimatorForceSensorDataHolder.calculateChecksum(estimatorChecksumCalculator);
      return estimatorChecksumCalculator.getValue();
   }
   
   private long calculateControllerChecksum()
   {
      controllerChecksumCalculator.reset();
      controllerChecksum.calculate();
      controllerForceSensorDataHolder.calculateChecksum(controllerChecksumCalculator);
      return controllerChecksumCalculator.getValue();
   }
   
   public void validate()
   {
      if(checksum != calculateControllerChecksum())
      {
         throw new RuntimeException("Controller checksum doesn't match estimator checksum.");
      }
   }

   public static class Builder implements us.ihmc.concurrent.Builder<IntermediateEstimatorStateHolder>
   {

      private final WholeBodyControlParameters robotModel;
      private final RigidBody estimatorRootJoint;
      private final RigidBody controllerRootJoint;

      private final ForceSensorDataHolder estimatorForceSensorDataHolder;
      private final ForceSensorDataHolder controllerForceSensorDataHolder;
      
      private final RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap;
      private final RawJointSensorDataHolderMap controllerRawJointSensorDataHolderMap;


      public Builder(WholeBodyControlParameters robotModel, RigidBody estimatorRootJoint, RigidBody controllerRootJoint,
            ForceSensorDataHolder estimatorForceSensorDataHolder, ForceSensorDataHolder controllerForceSensorDataHolder,
            RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap, RawJointSensorDataHolderMap controllerRawJointSensorDataHolderMap)
      {
         this.robotModel = robotModel;
         this.estimatorRootJoint = estimatorRootJoint;
         this.controllerRootJoint = controllerRootJoint;
         this.estimatorForceSensorDataHolder = estimatorForceSensorDataHolder;
         this.controllerForceSensorDataHolder = controllerForceSensorDataHolder;
         this.estimatorRawJointSensorDataHolderMap = estimatorRawJointSensorDataHolderMap;
         this.controllerRawJointSensorDataHolderMap = controllerRawJointSensorDataHolderMap;
      }

      @Override
      public IntermediateEstimatorStateHolder newInstance()
      {
         return new IntermediateEstimatorStateHolder(robotModel, estimatorRootJoint, controllerRootJoint, estimatorForceSensorDataHolder,
               controllerForceSensorDataHolder, estimatorRawJointSensorDataHolderMap, controllerRawJointSensorDataHolderMap);
      }

   }

}
