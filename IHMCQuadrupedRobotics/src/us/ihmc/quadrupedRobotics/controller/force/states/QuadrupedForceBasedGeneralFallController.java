package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedForceBasedGeneralFallController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedForceBasedGeneralFallController.class.getSimpleName());

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 2.0);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleParameter stanceLengthParameter = parameterFactory.createDouble("stanceLength", 1.0);
   private final DoubleParameter stanceWidthParameter = parameterFactory.createDouble("stanceWidth", 0.7);
   private final DoubleParameter stanceHeightParameter = parameterFactory.createDouble("stanceHeight", 0.40);
   private final DoubleParameter stanceXOffsetParameter = parameterFactory.createDouble("stanceXOffset", 0.05);
   private final DoubleParameter stanceYOffsetParameter = parameterFactory.createDouble("stanceYOffset", 0.0);
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory.createDoubleArray("solePositionProportionalGains", 1000, 1000, 1000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 10, 10, 10);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);

   private final DoubleYoVariable robotTime;
   private final QuadrupedReferenceFrames referenceFrames;

   // Sole trajectories
   private final TimeInterval trajectoryTimeInterval = new TimeInterval();
   private final FramePoint finalSolePosition = new FramePoint();
   private final QuadrantDependentList<ThreeDoFMinimumJerkTrajectory> solePositionTrajectories = new QuadrantDependentList<>();

   // Feedback controller
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionController.Setpoints solePositionControllerSetpoints;

   // Task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   public enum fallBehaviors{
      FREEZE, GO_HOME_Z, GO_HOME_XYZ
   }
   private final EnumYoVariable<fallBehaviors> fallBehavior = EnumYoVariable.create("fallBehaviors", fallBehaviors.class, registry);

   private final SDFFullQuadrupedRobotModel fullRobotModel;
   public QuadrupedForceBasedGeneralFallController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox)
   {
      this.robotTime = environment.getRobotTimestamp();
      this.referenceFrames = controllerToolbox.getReferenceFrames();

      // Feedback controller
      solePositionController = controllerToolbox.getSolePositionController();
      solePositionControllerSetpoints = new QuadrupedSolePositionController.Setpoints();

      // Task space controller
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionTrajectories.set(quadrant, new ThreeDoFMinimumJerkTrajectory());
      }
      fullRobotModel = environment.getFullRobotModel();
      fallBehavior.set(fallBehaviors.FREEZE);
      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      updateEstimates();

      double currentTime = robotTime.getDoubleValue();
      trajectoryTimeInterval.setInterval(0, trajectoryTimeParameter.get());
      trajectoryTimeInterval.shiftInterval(currentTime);

      // Initialize sole trajectories
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         ThreeDoFMinimumJerkTrajectory trajectory = solePositionTrajectories.get(quadrant);

         FramePoint initialSolePosition = taskSpaceEstimates.getSolePosition(quadrant);
         initialSolePosition.changeFrame(referenceFrames.getBodyFrame());

         computeFinalSolePosition(quadrant, finalSolePosition);
         finalSolePosition.changeFrame(referenceFrames.getBodyFrame());

         trajectory.initializeTrajectory(initialSolePosition, finalSolePosition, trajectoryTimeInterval);
      }

      // Initialize sole position controller
      solePositionControllerSetpoints.initialize(taskSpaceEstimates);
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.getGains(quadrant).setProportionalGains(solePositionProportionalGainsParameter.get());
         solePositionController.getGains(quadrant).setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
         solePositionController.getGains(quadrant).setDerivativeGains(solePositionDerivativeGainsParameter.get());
      }
      solePositionController.reset();

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();
   }

   private void computeFinalSolePosition(RobotQuadrant quadrant, FramePoint finalSolePosition)
   {
      finalSolePosition.setToZero(referenceFrames.getBodyFrame());
      switch(fallBehavior.getEnumValue()){
      case GO_HOME_XYZ:
         finalSolePosition.add(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.get() / 2.0), 0.0, 0.0);
         finalSolePosition.add(0.0, quadrant.getSide().negateIfRightSide(stanceWidthParameter.get() / 2.0), 0.0);
         finalSolePosition.add(stanceXOffsetParameter.get(), stanceYOffsetParameter.get(), -stanceHeightParameter.get());
         break;
      case GO_HOME_Z:
         finalSolePosition.add(taskSpaceEstimates.getSolePosition(quadrant).getX(),taskSpaceEstimates.getSolePosition(quadrant).getY(),-stanceHeightParameter.get());
         break;
      default:
         finalSolePosition.add(taskSpaceEstimates.getSolePosition(quadrant).getX(),taskSpaceEstimates.getSolePosition(quadrant).getY(),taskSpaceEstimates.getSolePosition(quadrant).getZ());
         break;
      }
   }

   @Override
   public ControllerEvent process()
   {
      updateEstimates();
      updateSetpoints();

      return isMotionExpired() ? ControllerEvent.DONE : null;
   }

   private void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   private void updateSetpoints()
   {
      double currentTime = robotTime.getDoubleValue();

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         // Compute the sole position setpoint along the minimum jerk trajectory.
         ThreeDoFMinimumJerkTrajectory trajectory = solePositionTrajectories.get(quadrant);
         trajectory.computeTrajectory(currentTime);

         trajectory.getPosition(solePositionControllerSetpoints.getSolePosition(quadrant));
      }

      solePositionController.compute(taskSpaceControllerCommands.getSoleForce(), solePositionControllerSetpoints, taskSpaceEstimates);
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }

   private boolean isMotionExpired()
   {
      double currentTime = robotTime.getDoubleValue();
      return currentTime > trajectoryTimeInterval.getEndTime();
   }

   @Override
   public void onExit()
   {
   }
}
