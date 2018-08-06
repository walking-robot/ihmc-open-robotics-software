package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedBodyHeightCommand;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDGains;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedCenterOfMassHeightManager
{
   private static final double minimumTimeRemaining = 0.01;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleParameter initializationDuration = new DoubleParameter("heightInitializationDuration", registry, 0.5);

   private final YoDouble controllerTime;

   private final YoBoolean controlBodyHeight = new YoBoolean("controlBodyHeight", registry);
   private final YoBoolean heightCommandHasBeenReceived = new YoBoolean("heightCommandHasBeenReceived", registry);

   private final MovingReferenceFrame bodyFrame;
   private final ReferenceFrame centerOfMassFrame;

   private final QuadrupedControllerToolbox controllerToolbox;

   private final ReferenceFrame supportFrame;
   private final MultipleWaypointsPositionTrajectoryGenerator centerOfMassHeightTrajectory;

   private final YoDouble currentDesiredHeightInWorld = new YoDouble("currentDesiredHeightInWorld", registry);
   private final YoDouble currentDesiredVelocityInWorld = new YoDouble("currentDesiredVelocityInWorld", registry);
   private final YoDouble upcomingDesiredHeightInWorld = new YoDouble("upcomingDesiredHeightInWorld", registry);

   private final List<QuadrupedTimedStep> nextSteps = new ArrayList<>();

   private final PIDController linearMomentumZPDController;

   private final FramePoint3D currentPosition = new FramePoint3D();
   private final FrameVector3D currentVelocity = new FrameVector3D();

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D desiredAcceleration = new FrameVector3D();

   private final ParameterizedPIDGains comPositionGainsParameter;

   private final YoDouble currentHeightInWorld;
   private final YoDouble desiredHeightInWorld;
   private final YoDouble desiredVelocityInWorld;
   private final YoDouble currentVelocityInWorld;

   private final FramePoint3D nominalPosition;
   private final FrameVector3D nominalVelocity;

   private final double controlDT;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final GroundPlaneEstimator upcomingGroundPlaneEstimator;

   public QuadrupedCenterOfMassHeightManager(QuadrupedControllerToolbox controllerToolbox, QuadrupedPhysicalProperties physicalProperties,
                                             YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.controllerTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.controlDT = controllerToolbox.getRuntimeEnvironment().getControlDT();

      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      upcomingGroundPlaneEstimator = controllerToolbox.getUpcomingGroundPlaneEstimator();
      supportFrame = groundPlaneEstimator.getGroundPlaneFrame();
      bodyFrame = referenceFrames.getBodyFrame();
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      PIDGains defaultComPositionGains = new PIDGains();
      defaultComPositionGains.setKp(50.0);
      defaultComPositionGains.setKd(5.0);
      comPositionGainsParameter = new ParameterizedPIDGains("_comHeight", defaultComPositionGains, registry);
      linearMomentumZPDController = new PIDController("linearMomentumZPDController", registry);

      controlBodyHeight.set(true);
      heightCommandHasBeenReceived.set(false);

      nominalPosition = new FramePoint3D(supportFrame, 0.0, 0.0, physicalProperties.getNominalCoMHeight());
      nominalVelocity = new FrameVector3D(supportFrame);

      centerOfMassHeightTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("centerOfMassHeight", supportFrame, registry);

      currentHeightInWorld = new YoDouble("currentHeightInWorld", registry);
      currentVelocityInWorld = new YoDouble("currentVelocityInWorld", registry);
      desiredHeightInWorld = new YoDouble("desiredHeightInWorld", registry);
      desiredVelocityInWorld = new YoDouble("desiredVelocityInWorld", registry);

      parentRegistry.addChild(registry);
   }

   public void handleBodyHeightCommand(QuadrupedBodyHeightCommand command)
   {
      controlBodyHeight.set(command.controlBodyHeight());

      double currentTime = controllerTime.getDoubleValue();
      double timeShift = command.isExpressedInAbsoluteTime() ? 0.0 : currentTime;
      EuclideanTrajectoryControllerCommand euclideanTrajectory = command.getEuclideanTrajectory();
      euclideanTrajectory.getTrajectoryPointList().addTimeOffset(timeShift);

      if (euclideanTrajectory.getTrajectoryPoint(0).getTime() > 1.0e-5 + currentTime)
      {
         centerOfMassHeightTrajectory.getPosition(desiredPosition);
         desiredVelocity.setToZero(worldFrame);

         desiredPosition.changeFrame(supportFrame);
         desiredVelocity.changeFrame(supportFrame);

         centerOfMassHeightTrajectory.clear();
         centerOfMassHeightTrajectory.appendWaypoint(currentTime, desiredPosition, desiredVelocity);
      }
      else
      {
         centerOfMassHeightTrajectory.clear();
      }

      for (int i = 0; i < euclideanTrajectory.getNumberOfTrajectoryPoints(); i++)
      {
         FrameEuclideanTrajectoryPoint trajectoryPoint = euclideanTrajectory.getTrajectoryPoint(i);
         trajectoryPoint.changeFrame(supportFrame);

         centerOfMassHeightTrajectory.appendWaypoint(trajectoryPoint);
      }

      centerOfMassHeightTrajectory.initialize();
      heightCommandHasBeenReceived.set(true);
   }

   public void initialize()
   {
      if (!heightCommandHasBeenReceived.getBooleanValue())
      {
         computeCurrentState();

         currentPosition.changeFrame(supportFrame);
         currentVelocity.changeFrame(supportFrame);

         double startTime = controllerTime.getDoubleValue();
         double endTime = startTime + initializationDuration.getValue();
         centerOfMassHeightTrajectory.clear();
         centerOfMassHeightTrajectory.appendWaypoint(startTime, currentPosition, currentVelocity);
         centerOfMassHeightTrajectory.appendWaypoint(endTime, nominalPosition, nominalVelocity);
         centerOfMassHeightTrajectory.initialize();
      }

      nextSteps.clear();
   }

   public void setActiveSteps(List<? extends QuadrupedTimedStep> activeSteps)
   {
      double earliestEndTime = Double.POSITIVE_INFINITY;
      nextSteps.clear();

      for (int i = 0; i < activeSteps.size(); i++)
      {
         double endTime = activeSteps.get(i).getTimeInterval().getEndTime();
         if (endTime < earliestEndTime)
            earliestEndTime = endTime;
      }

      for (int i = 0; i < activeSteps.size(); i++)
      {
         QuadrupedTimedStep step = activeSteps.get(i);
         if (step.getTimeInterval().getEndTime() == earliestEndTime)
         {
            nextSteps.add(step);
         }
      }
   }

   private final FramePoint3D tempDesiredPosition = new FramePoint3D();

   public void update()
   {
      centerOfMassHeightTrajectory.compute(controllerTime.getDoubleValue());
      centerOfMassHeightTrajectory.getLinearData(desiredPosition, desiredVelocity, desiredAcceleration);

      // get the potential XY translation in height
      double desiredZHeight = desiredPosition.getZ();
      if (controlBodyHeight.getBooleanValue())
      {
         desiredPosition.setToZero(bodyFrame);
         tempDesiredPosition.setToZero(bodyFrame);
      }
      else
      {
         desiredPosition.setToZero(centerOfMassFrame);
         tempDesiredPosition.setToZero(centerOfMassFrame);
      }
      desiredPosition.changeFrame(supportFrame);
      desiredPosition.setZ(desiredZHeight);

      desiredPosition.changeFrame(worldFrame);
      desiredVelocity.changeFrame(worldFrame);



      // compute desired height in upcoming support frame
      tempDesiredPosition.changeFrame(upcomingGroundPlaneEstimator.getGroundPlaneFrame());
      tempDesiredPosition.setZ(desiredZHeight);
      tempDesiredPosition.changeFrame(worldFrame);


      currentDesiredHeightInWorld.set(desiredPosition.getZ());
      currentDesiredVelocityInWorld.set(desiredVelocity.getZ());

      upcomingDesiredHeightInWorld.set(tempDesiredPosition.getZ());

      // blend between the current desired height and the upcoming desired height
      double alpha = getHeightBlendingFactor();
      double blendedHeight = InterpolationTools
            .linearInterpolate(currentDesiredHeightInWorld.getDoubleValue(), upcomingDesiredHeightInWorld.getDoubleValue(), alpha);
      double blendedHeightVelocity = currentDesiredVelocityInWorld.getDoubleValue() + getBlendedHeightVelocity();

      /*
      if (upcomingDesiredHeightInWorld.getDoubleValue() < currentDesiredHeightInWorld.getDoubleValue()) // stepping down
      {
         if (desiredHeightInWorld.getDoubleValue() < blendedHeight)
         {
            blendedHeight = desiredHeightInWorld.getDoubleValue();
            blendedHeightVelocity = currentDesiredVelocityInWorld.getDoubleValue();
         }
      }
      else if (upcomingDesiredHeightInWorld.getDoubleValue() > currentDesiredHeightInWorld.getDoubleValue()) // stepping up
      {
         if (desiredHeightInWorld.getDoubleValue() > blendedHeight)
         {
            blendedHeight = desiredHeightInWorld.getDoubleValue();
            blendedHeightVelocity = currentDesiredVelocityInWorld.getDoubleValue();
         }
      }
      */

      desiredHeightInWorld.set(blendedHeight);
      desiredVelocityInWorld.set(blendedHeightVelocity);

      computeCurrentState();
   }

   public double getDesiredHeight(ReferenceFrame referenceFrame)
   {
      desiredPosition.changeFrame(referenceFrame);
      return desiredPosition.getZ();
   }

   public double computeDesiredCenterOfMassHeightAcceleration()
   {
      linearMomentumZPDController.setGains(comPositionGainsParameter);
      return linearMomentumZPDController
            .compute(currentHeightInWorld.getDoubleValue(), desiredHeightInWorld.getDoubleValue(), currentVelocityInWorld.getDoubleValue(),
                     desiredVelocityInWorld.getDoubleValue(), controlDT);
   }

   private double getHeightBlendingFactor()
   {
      if (nextSteps.size() > 0)
      {
         double duration = nextSteps.get(0).getTimeInterval().getDuration();
         double startTime = nextSteps.get(0).getTimeInterval().getStartTime();
         double timeInState = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp().getDoubleValue() - startTime;

         return MathTools.clamp(timeInState / duration, 0.0, 1.0);
      }
      else
      {
         return 0.0;
      }
   }

   private double getBlendedHeightVelocity()
   {
      if (nextSteps.size() > 0)
      {
         double timeRemainingInState =
               nextSteps.get(0).getTimeInterval().getEndTime() - controllerToolbox.getRuntimeEnvironment().getRobotTimestamp().getDoubleValue();
         timeRemainingInState = Math.max(timeRemainingInState, minimumTimeRemaining);
         double heightDifference = upcomingDesiredHeightInWorld.getDoubleValue() - desiredHeightInWorld.getDoubleValue();

         return heightDifference / timeRemainingInState;
      }
      else
      {
         return 0.0;
      }
   }

   private void computeCurrentState()
   {
      if (controlBodyHeight.getBooleanValue())
      {
         currentPosition.setToZero(bodyFrame);
         bodyFrame.getTwistOfFrame().getLinearVelocityOfPointFixedInBodyFrame(currentVelocity, currentPosition);
      }
      else
      {
         currentPosition.setToZero(centerOfMassFrame);
         currentVelocity.setIncludingFrame(controllerToolbox.getCoMVelocityEstimate());
      }

      currentPosition.changeFrame(worldFrame);
      currentVelocity.changeFrame(worldFrame);

      currentHeightInWorld.set(currentPosition.getZ());
      currentVelocityInWorld.set(currentVelocity.getZ());
   }
}
