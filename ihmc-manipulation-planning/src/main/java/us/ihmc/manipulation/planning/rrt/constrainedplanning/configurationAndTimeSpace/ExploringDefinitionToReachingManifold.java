package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.List;

import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;

public class ExploringDefinitionToReachingManifold extends ExploringDefinition
{

   public ExploringDefinitionToReachingManifold(List<WaypointBasedTrajectoryCommand> endEffectorTrajectories, List<ReachingManifoldCommand> manifolds,
                                                List<RigidBodyExplorationConfigurationCommand> explorationConfigurations)
   {
      super(endEffectorTrajectories, explorationConfigurations);
      // TODO Auto-generated constructor stub
   }

   @Override
   public double getExploringProgress(SpatialNode node)
   {
      // TODO Auto-generated method stub
      return 0;
   }

}