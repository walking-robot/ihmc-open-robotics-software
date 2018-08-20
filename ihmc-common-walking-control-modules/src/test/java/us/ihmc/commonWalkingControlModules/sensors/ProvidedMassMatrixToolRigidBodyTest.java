package us.ihmc.commonWalkingControlModules.sensors;

import static org.junit.Assert.assertTrue;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class ProvidedMassMatrixToolRigidBodyTest
{
   private final YoVariableRegistry registry = new YoVariableRegistry("ProvidedMassMatrixToolRigidBodyTest");
   private final RobotSide robotSide = RobotSide.LEFT;
   private final double gravity = 9.81;
   private final double mass = 2.0;

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testprovidedMassMatrixToolRigidBody()
   {
      FullHumanoidRobotModel fullRobotModel = getFullRobotModel();

      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();

      ProvidedMassMatrixToolRigidBody providedMassMatrixToolRigidBody = new ProvidedMassMatrixToolRigidBody(robotSide, fullRobotModel, gravity, registry, null);
      providedMassMatrixToolRigidBody.setMass(mass);

      SpatialAccelerationVector handSpatialAccelerationVector = new SpatialAccelerationVector(elevatorFrame, elevatorFrame, elevatorFrame);
      Wrench toolWrench = new Wrench();

      providedMassMatrixToolRigidBody.control(handSpatialAccelerationVector, toolWrench);

      toolWrench.changeFrame(ReferenceFrame.getWorldFrame());

      assertTrue(toolWrench.getLinearPart().epsilonEquals(new Vector3D(0.0, 0.0, -mass * gravity), 10e-5));
   }

   public abstract FullHumanoidRobotModel getFullRobotModel();
}
