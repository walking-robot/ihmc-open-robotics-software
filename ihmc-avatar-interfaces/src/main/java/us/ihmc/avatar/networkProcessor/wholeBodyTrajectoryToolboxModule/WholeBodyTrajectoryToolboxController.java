package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
import us.ihmc.manipulation.planning.exploringSpatial.ExploringDefinition;
import us.ihmc.manipulation.planning.exploringSpatial.ExploringProgressVisualizer;
import us.ihmc.manipulation.planning.exploringSpatial.SpatialData;
import us.ihmc.manipulation.planning.exploringSpatial.SpatialNode;
import us.ihmc.manipulation.planning.exploringSpatial.SpatialNodePlotter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class WholeBodyTrajectoryToolboxController extends ToolboxController
{
   private static final boolean VERBOSE = true;
   private static final int DEFAULT_NUMBER_OF_ITERATIONS_FOR_SHORTCUT_OPTIMIZATION = 10;
   private static final int DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE = 500;
   private static final int DEFAULT_NUMBER_OF_INITIAL_GUESSES_VALUE = 100;
   private static final int DEFAULT_NUMBER_OF_WAYPOINTS_TO_GOAL = 30;

   private final HumanoidKinematicsSolver humanoidKinematicsSolver;

   private final WholeBodyTrajectoryToolboxOutputStatus toolboxSolution;

   private List<RigidBodyExplorationConfigurationCommand> rigidBodyCommands = null;
   private List<WaypointBasedTrajectoryCommand> trajectoryCommands = null;
   private List<ReachingManifoldCommand> manifoldCommands = null;

   /*
    * YoVariables
    */
   private final YoInteger currentNumberOfIterations = new YoInteger("currentNumberOfIterations", registry);
   private final YoInteger maximumExpansionSize = new YoInteger("maximumExpansionSize", registry);
   private final YoInteger desiredNumberOfInitialGuesses = new YoInteger("desiredNumberOfInitialGuesses", registry);

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoBoolean isValidNode = new YoBoolean("isValidNode", registry);
   private final YoDouble exploringProgress = new YoDouble("exploringProgress", registry);

   /*
    * Visualizer
    */
   private boolean visualize;

   private SpatialNode visualizedNode;

   private final KinematicsToolboxOutputConverter configurationConverter;
   private final KinematicsToolboxOutputStatus initialConfiguration = new KinematicsToolboxOutputStatus();
   private final AtomicReference<RobotConfigurationData> currentRobotConfigurationDataReference = new AtomicReference<>(null);

   private FullHumanoidRobotModel visualizedFullRobotModel;

   private ExploringProgressVisualizer treeStateVisualizer;

   private SpatialNodePlotter nodePlotter;

   private final CommandInputManager commandInputManager;

   private final List<SpatialNode> path = new ArrayList<>();

   private ExploringDefinition exploringDefinition;

   private final StateMachine<ToolboxStateName, WholeBodyTrajectoryToolboxState> stateMachine;

   public enum ToolboxStateName
   {
      DEFAULT_TRAJECTORY_TYRIAL, EXPAND_TREE, SHORTCUT_PATH
   }

   private final WholeBodyTrajectoryToolboxState defaultTrialState;
   private final WholeBodyTrajectoryToolboxState exploringState;
   private final WholeBodyTrajectoryToolboxState shorcutState;

   public WholeBodyTrajectoryToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel, CommandInputManager commandInputManager,
                                               StatusMessageOutputManager statusOutputManager, YoVariableRegistry registry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry, boolean visualize)
   {
      super(statusOutputManager, registry);
      this.commandInputManager = commandInputManager;

      visualizedFullRobotModel = fullRobotModel;
      isDone.set(false);

      this.visualize = visualize;
      if (visualize)
         treeStateVisualizer = new ExploringProgressVisualizer("TreeStateVisualizer", "VisualizerGraphicsList", yoGraphicsListRegistry, registry);
      else
         treeStateVisualizer = null;

      humanoidKinematicsSolver = new HumanoidKinematicsSolver(drcRobotModel, yoGraphicsListRegistry, registry);

      toolboxSolution = new WholeBodyTrajectoryToolboxOutputStatus();
      toolboxSolution.setDestination(-1);

      initialConfiguration.setJointNameHash(setJointNameHashFromRobotModelWithExcludingHands());

      configurationConverter = new KinematicsToolboxOutputConverter(drcRobotModel);

      defaultTrialState = new DefaultTrialState(DEFAULT_NUMBER_OF_WAYPOINTS_TO_GOAL);

      exploringState = new ExploringState(DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE);
      shorcutState = new ShorcutState(DEFAULT_NUMBER_OF_ITERATIONS_FOR_SHORTCUT_OPTIMIZATION);
      stateMachine = setupStateMachine();
   }

   private int setJointNameHashFromRobotModelWithExcludingHands()
   {
      return (int) NameBasedHashCodeTools.computeArrayHashCode(FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel));
   }

   private StateMachine<ToolboxStateName, WholeBodyTrajectoryToolboxState> setupStateMachine()
   {
      StateMachineFactory<ToolboxStateName, WholeBodyTrajectoryToolboxState> factory = new StateMachineFactory<>(ToolboxStateName.class);

      factory.addState(ToolboxStateName.DEFAULT_TRAJECTORY_TYRIAL, defaultTrialState);
      factory.addTransition(ToolboxStateName.DEFAULT_TRAJECTORY_TYRIAL, ToolboxStateName.EXPAND_TREE,
                            t -> defaultTrialState.hasFail() && defaultTrialState.isDone(t));
      factory.addTransition(ToolboxStateName.DEFAULT_TRAJECTORY_TYRIAL, ToolboxStateName.SHORTCUT_PATH,
                            t -> !defaultTrialState.hasFail() && defaultTrialState.isDone(t));
      factory.addStateAndDoneTransition(ToolboxStateName.EXPAND_TREE, exploringState, ToolboxStateName.SHORTCUT_PATH);
      factory.addState(ToolboxStateName.SHORTCUT_PATH, shorcutState);

      return factory.build(ToolboxStateName.DEFAULT_TRAJECTORY_TYRIAL);
   }

   @Override
   protected void updateInternal() throws InterruptedException, ExecutionException
   {
      currentNumberOfIterations.increment();
      stateMachine.doActionAndTransition();

      if (stateMachine.getCurrentStateKey() != ToolboxStateName.SHORTCUT_PATH)
      {
         updateVisualizerRobotConfiguration();
         updateVisualizers();
      }

      if (!stateMachine.isCurrentStateTerminal() && stateMachine.getCurrentState().isDone(0.0))
      {
         stateMachine.getCurrentState().onExit();
         terminateToolboxController();
      }
   }

   private void setFailureOnOutputStatus(WholeBodyTrajectoryToolboxOutputStatus outputStatusToPack, ToolboxStateName toolboxStateName)
   {
      switch (toolboxStateName)
      {
      case DEFAULT_TRAJECTORY_TYRIAL:
         outputStatusToPack.setPlanningResult(1);
         break;
      case EXPAND_TREE:
         outputStatusToPack.setPlanningResult(2);
         break;
      case SHORTCUT_PATH:
         outputStatusToPack.setPlanningResult(3);
         break;
      default:
         break;
      }
   }

   private void setOutputStatus(WholeBodyTrajectoryToolboxOutputStatus outputStatusToPack, List<SpatialNode> path)
   {
      outputStatusToPack.setPlanningResult(4);
      MessageTools.copyData(path.stream().map(SpatialNode::getConfiguration).toArray(size -> new KinematicsToolboxOutputStatus[size]),
                            outputStatusToPack.getRobotConfigurations());
      outputStatusToPack.getTrajectoryTimes().reset();
      outputStatusToPack.getTrajectoryTimes().add(path.stream().mapToDouble(SpatialNode::getTime).toArray());
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);

      boolean success = updateConfiguration();
      if (!success)
         return false;

      if (!commandInputManager.isNewCommandAvailable(WaypointBasedTrajectoryCommand.class))
         return false;

      if (!commandInputManager.isNewCommandAvailable(RigidBodyExplorationConfigurationCommand.class))
         return false;

      if (!commandInputManager.isNewCommandAvailable(ReachingManifoldCommand.class))
         return false;

      trajectoryCommands = commandInputManager.pollNewCommands(WaypointBasedTrajectoryCommand.class);
      rigidBodyCommands = commandInputManager.pollNewCommands(RigidBodyExplorationConfigurationCommand.class);
      manifoldCommands = commandInputManager.pollNewCommands(ReachingManifoldCommand.class);

      // ******************************************************************************** //
      // Convert command into WholeBodyTrajectoryToolboxData.
      // ******************************************************************************** //
      configurationConverter.updateFullRobotModel(initialConfiguration);

      // ExploringDefinition spatialDefinition;
      if (trajectoryCommands != null)
      {
         exploringDefinition = new ExploringDefinition(trajectoryCommands, rigidBodyCommands, manifoldCommands);
         exploringState.setMaximumNumberOfUpdate(maximumExpansionSize.getIntegerValue());
      }
      else
         return false;

      nodePlotter = new SpatialNodePlotter(exploringDefinition, visualize);
      validNodes.clear();
      inValidNodes.clear();
      stateMachine.resetToInitialState();
      currentNumberOfIterations.set(0);
      return true;
   }

   private boolean updateConfiguration()
   {
      int newMaxExpansionSize = -1;
      int newNumberOfInitialGuesses = -1;
      KinematicsToolboxOutputStatus newInitialConfiguration = null;

      // pull commands. configuration, trajectory, reaching.
      if (commandInputManager.isNewCommandAvailable(WholeBodyTrajectoryToolboxConfigurationCommand.class))
      {
         WholeBodyTrajectoryToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(WholeBodyTrajectoryToolboxConfigurationCommand.class);

         newMaxExpansionSize = command.getMaximumExpansionSize();
         newNumberOfInitialGuesses = command.getNumberOfInitialGuesses();

         if (command.hasInitialConfiguration())
            newInitialConfiguration = command.getInitialConfiguration();
      }

      // toolbox setup parameters.
      if (newMaxExpansionSize > 0)
         maximumExpansionSize.set(newMaxExpansionSize);
      else
         maximumExpansionSize.set(DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE);

      if (newNumberOfInitialGuesses > 0)
         desiredNumberOfInitialGuesses.set(newNumberOfInitialGuesses);
      else
         desiredNumberOfInitialGuesses.set(DEFAULT_NUMBER_OF_INITIAL_GUESSES_VALUE);

      // set initial configuration of full robot model in toolbox.
      if (newInitialConfiguration != null)
      {
         initialConfiguration.set(newInitialConfiguration);
         initialConfiguration.setJointNameHash(setJointNameHashFromRobotModelWithExcludingHands());
         return true;
      }

      RobotConfigurationData currentRobotConfiguration = currentRobotConfigurationDataReference.getAndSet(null);
      if (currentRobotConfiguration == null)
         return false;

      initialConfiguration.getDesiredRootOrientation().set(currentRobotConfiguration.getRootOrientation());
      initialConfiguration.getDesiredRootTranslation().set(currentRobotConfiguration.getRootTranslation());

      initialConfiguration.setJointNameHash(setJointNameHashFromRobotModelWithExcludingHands());
      MessageTools.copyData(currentRobotConfiguration.getJointAngles(), initialConfiguration.getDesiredJointAngles());

      return true;
   }

   private void terminateToolboxController()
   {
      toolboxSolution.setDestination(PacketDestination.BEHAVIOR_MODULE.ordinal());

      reportMessage(toolboxSolution);

      if (!VERBOSE)
         nodePlotter.closeAll();

      isDone.set(true);
   }

   @Override
   protected boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   /**
    * update validity of input node.
    */
   private boolean updateValidity(SpatialNode node)
   {
      if (node.getParent() != null && node.getParent().getConfiguration() != null)
      {
         humanoidKinematicsSolver.setInitialConfiguration(node.getParent().getConfiguration());
      }
      else
      {
         humanoidKinematicsSolver.setInitialConfiguration(initialConfiguration);
      }

      humanoidKinematicsSolver.initialize();
      humanoidKinematicsSolver.submit(exploringDefinition.createMessages(node));
      /*
       * result
       */
      boolean success = humanoidKinematicsSolver.solve();

      node.setConfiguration(humanoidKinematicsSolver.getSolution());
      node.setValidity(success);

      return success;
   }

   /**
    * set fullRobotModel.
    */
   private void updateVisualizerRobotConfiguration()
   {
      MessageTools.unpackDesiredJointState(visualizedNode.getConfiguration(), visualizedFullRobotModel.getRootJoint(),
                                           FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel));
   }

   /**
    * update visualizers.
    */
   private void updateVisualizers()
   {
      isValidNode.set(visualizedNode.isValid());

      if (visualize && visualizedNode != null)
      {
         treeStateVisualizer.setRecentProgress(exploringDefinition.getExploringProgress(visualizedNode));
         treeStateVisualizer.setDesiredNodeValidity(visualizedNode.isValid());
         treeStateVisualizer.updateVisualizer();
      }
   }

   /**
    * oneTime shortcut : try to make a shortcut from index to index+2
    */
   private boolean updateShortcutPath(List<SpatialNode> path, int index)
   {
      // check out when index is over the size.
      if (index > path.size() - 3)
      {
         return false;
      }

      SpatialNode nodeDummy = new SpatialNode(path.get(index + 1));

      nodeDummy.setParent(path.get(index));
      nodeDummy.interpolate(path.get(index), path.get(index + 2), 0.5);

      updateValidity(nodeDummy);

      if (nodeDummy.isValid())
      {
         path.get(index + 1).interpolate(path.get(index), path.get(index + 2), 0.5);
         path.get(index + 1).setConfiguration(nodeDummy.getConfiguration());

         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * return distance of the paths before and after shortcut.
    */
   private double updateShortcutPath(List<SpatialNode> path)
   {
      ArrayList<SpatialNode> pathBeforeShortcut = new ArrayList<SpatialNode>();

      for (int i = 0; i < path.size(); i++)
      {
         pathBeforeShortcut.add(new SpatialNode(path.get(i)));
      }

      for (int i = 0; i < path.size(); i++)
      {
         if (updateShortcutPath(path, i))
            ;
      }

      double distance = 0.0;
      for (int i = 0; i < path.size(); i++)
      {
         distance += computeDistance(pathBeforeShortcut.get(i), path.get(i));
      }

      return distance / path.size();
   }

   /**
    * Compute distance from this to other.
    */
   private double computeDistance(SpatialNode one, SpatialNode other)
   {
      double timeDistance = timeWeight * Math.abs(one.getTime() - other.getTime());
      double positionDistance = positionWeight * one.getSpatialData().getPositionDistance(other.getSpatialData());
      double orientationDistance = orientationWeight * one.getSpatialData().getOrientationDistance(other.getSpatialData());

      double distance = timeDistance + positionDistance + orientationDistance;
      return distance;
   }

   private void submitDesiredNodeToToolbox(SpatialNode node)
   {
      updateValidity(node);
      visualizedNode = node;
      nodePlotter.update(node, 1);
   }

   private List<SpatialNode> getPath()
   {
      List<SpatialNode> path = new ArrayList<>();
      path.clear();

      List<SpatialNode> revertedPath = new ArrayList<SpatialNode>();
      SpatialNode currentNode = validNodes.get(validNodes.size() - 1);
      revertedPath.add(currentNode);

      while (true)
      {
         SpatialNode parentNode = currentNode.getParent();
         if (parentNode != null)
         {
            revertedPath.add(parentNode);
            currentNode = parentNode;
         }
         else
            break;
      }

      int revertedPathSize = revertedPath.size();

      for (int i = 0; i < revertedPathSize; i++)
      {
         path.add(revertedPath.get(revertedPathSize - 1 - i));
      }
      return path;
   }

   private SpatialNode findNearestValidNodeToRandomNode(SpatialNode randomNode)
   {
      double distanceToNearestNode = Double.MAX_VALUE;
      SpatialNode nearestNode = null;

      for (int i = 0; i < validNodes.size(); i++)
      {
         if (randomNode.getTime() < validNodes.get(i).getTime())
            continue;

         double distance = validNodes.get(i).computeDistanceWithinMaxDistance(timeWeight, positionWeight, orientationWeight, randomNode, maxTimeInterval,
                                                                              maxPositionDistance, maxOrientationDistance);
         if (distance < distanceToNearestNode)
         {
            distanceToNearestNode = distance;
            nearestNode = validNodes.get(i);
         }
      }

      return nearestNode;
   }

   void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      currentRobotConfigurationDataReference.set(newConfigurationData);
   }

   public FullHumanoidRobotModel getSolverFullRobotModel()
   {
      return humanoidKinematicsSolver.getDesiredFullRobotModel();
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus takeNextData)
   {

   }

   private ArrayList<SpatialNode> validNodes = new ArrayList<SpatialNode>();
   private ArrayList<SpatialNode> inValidNodes = new ArrayList<SpatialNode>();

   private static final double timeWeight = 0.1;
   private static final double positionWeight = 1.0;
   private static final double orientationWeight = 1.0;

   private static final double maxTimeInterval = 1 / 2.0; // 1/ 2.0 is original
   private static final double maxPositionDistance = 0.05;
   private static final double maxOrientationDistance = Math.toRadians(5.0);

   private static final double progressSaturationThreshold = 0.9;

   private abstract class WholeBodyTrajectoryToolboxState implements State
   {
      private long startTime;

      public int numberOfUpdate;
      private int maximumNumberOfUpdate;

      public double lastStepProgress = 0.0;

      public WholeBodyTrajectoryToolboxState(int maximumNumberOfUpdate)
      {
         this.maximumNumberOfUpdate = maximumNumberOfUpdate;
      }

      public void setMaximumNumberOfUpdate(int value)
      {
         this.maximumNumberOfUpdate = value;
      }

      public void updateNodeLists(SpatialNode newNode)
      {
         if (newNode.isValid())
            validNodes.add(newNode);
         else
            inValidNodes.add(newNode);
      }

      @Override
      public void onEntry()
      {
         if (VERBOSE)
            System.out.println("onEntry " + getClass().getSimpleName() + " " + validNodes.size());

         lastStepProgress = 0.0;
         numberOfUpdate = 0;
         startTime = System.nanoTime();
      }

      @Override
      public void doAction(double timeInState)
      {
         numberOfUpdate++;

         SpatialNode desiredNode = createDesiredNode();

         if (desiredNode != null)
         {
            submitDesiredNodeToToolbox(desiredNode);
            updateNodeLists(desiredNode);
            exploringProgress.set(exploringDefinition.getExploringProgress(desiredNode));
            updateProgress(desiredNode);
         }
      }

      @Override
      public void onExit()
      {
         long endTime = System.nanoTime();
         double computationTime = Conversions.nanosecondsToSeconds(endTime - startTime);
         //         if (VERBOSE)
         //            System.out.println("onEntry " + getClass().getSimpleName() + " computationTime " + computationTime + " " + validNodes.size() + " " + numberOfUpdate);

         if (hasFail())
            setFailureOnOutputStatus(toolboxSolution, stateMachine.getCurrentStateKey());
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return numberOfUpdate >= maximumNumberOfUpdate || lastStepProgress >= 0.995;
      }

      abstract boolean hasFail();

      abstract void updateProgress(SpatialNode newNode);

      abstract SpatialNode createDesiredNode();
   }

   private class DefaultTrialState extends WholeBodyTrajectoryToolboxState
   {
      private int numberOfWayPoints;
      private int currentIndexOfWayPoint;

      public DefaultTrialState(int maximumNumberOfUpdate)
      {
         super(maximumNumberOfUpdate);
      }

      @Override
      public void onExit()
      {
         super.onExit();
         if (!hasFail())
         {
            path.clear();
            nodePlotter.update(getPath(), 2);
            path.addAll(getPath());
         }
      }

      @Override
      public void onEntry()
      {
         super.onEntry();
         numberOfWayPoints = (int) (exploringDefinition.getTrajectoryTime() / maxTimeInterval);
         currentIndexOfWayPoint = 0;
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return super.isDone(timeInState) || inValidNodes.size() > 0;
      }

      private boolean progressIsSaturated()
      {
         return lastStepProgress >= progressSaturationThreshold;
      }

      @Override
      boolean hasFail()
      {
         return inValidNodes.size() > 0;
      }

      @Override
      void updateProgress(SpatialNode newNode)
      {
         lastStepProgress = exploringDefinition.getExploringProgress(newNode);
      }

      @Override
      SpatialNode createDesiredNode()
      {
         SpatialNode desiredNode;

         if (progressIsSaturated())
         {
            desiredNode = exploringDefinition.createFinalSpatialNode(validNodes.get(validNodes.size() - 1));
         }
         else
         {
            double progress = currentIndexOfWayPoint / (double) numberOfWayPoints;
            SpatialData spatialData = exploringDefinition.createDefaultSpatialData();
            desiredNode = new SpatialNode(exploringDefinition.getTrajectoryTime() * progress, spatialData);
         }

         if (currentIndexOfWayPoint != 0)
            desiredNode.setParent(validNodes.get(currentIndexOfWayPoint - 1));

         currentIndexOfWayPoint++;
         return desiredNode;
      }
   }

   private class ExploringState extends WholeBodyTrajectoryToolboxState
   {
      private final static int maximumCountForWating = 500;
      private final static double timeCoefficient = 1.0;

      public ExploringState(int maximumNumberOfUpdate)
      {
         super(maximumNumberOfUpdate);
      }

      @Override
      public void onExit()
      {
         super.onExit();
         path.clear();
         nodePlotter.update(getPath(), 2);
         path.addAll(getPath());
      }

      @Override
      boolean hasFail()
      {
         if (lastStepProgress == 1.0)
            return false;
         else
            return true;
      }

      @Override
      SpatialNode createDesiredNode()
      {
         SpatialNode desiredNode;

         if (lastProgressIsSaturated())
            desiredNode = exploringDefinition.createFinalSpatialNode(validNodes.get(validNodes.size() - 1));
         else
            desiredNode = createRandomNode();

         return desiredNode;
      }

      @Override
      void updateProgress(SpatialNode newNode)
      {
         lastStepProgress = exploringDefinition.getExploringProgress(newNode);

         // allow round error for a method 'createFinalSpatialNode()' which is in ExploringDefinition.
         if (lastStepProgress > 0.995)
            lastStepProgress = 1.0;
      }

      private SpatialNode createRandomNode()
      {
         SpatialData randomSpatialData = null;
         SpatialNode randomNode = null;
         SpatialNode nearestNode = null;

         for (int i = 0; i < maximumCountForWating; i++)
         {
            // create random node.
            double nextDouble = WholeBodyTrajectoryToolboxMessageTools.random.nextDouble();
            double randomTime = exploringDefinition.getTrajectoryTime() * nextDouble * (1.0 + timeCoefficient * lastStepProgress);
            randomTime = Math.min(exploringDefinition.getTrajectoryTime(), randomTime);
            randomSpatialData = exploringDefinition.createRandomSpatialData();

            randomNode = new SpatialNode(randomTime, randomSpatialData);
            nearestNode = findNearestValidNodeToRandomNode(randomNode);
            if (nearestNode != null)
               break;
         }
         if (nearestNode == null) // System.out.println("could not find nearest node");
            return null;

         // create desired node.
         double timeGap = nearestNode.getTimeGap(randomNode);

         double alpha = 1.0;

         double timeStepToward;

         timeStepToward = Math.min(exploringDefinition.getTrajectoryTime() - nearestNode.getTime(), maxTimeInterval);

         alpha = Math.min(alpha, timeStepToward / timeGap);

         SpatialNode node = new SpatialNode(randomNode);
         node.interpolate(nearestNode, randomNode, alpha);
         node.setParent(nearestNode);

         return node;
      }

      private boolean lastProgressIsSaturated()
      {
         return lastStepProgress >= progressSaturationThreshold;
      }
   }

   private class ShorcutState extends WholeBodyTrajectoryToolboxState
   {
      public final double minimumAllowableShortcut = 0.001;
      public double pathDiff;

      public ShorcutState(int maximumNumberOfUpdate)
      {
         super(maximumNumberOfUpdate);
      }

      @Override
      public void onExit()
      {
         super.onExit();
         nodePlotter.update(path, 3);

         setOutputStatus(toolboxSolution, path);
      }

      @Override
      public void doAction(double timeInState)
      {
         super.doAction(timeInState);
         pathDiff = updateShortcutPath(path);
         updateProgress(null);
      }

      @Override
      boolean hasFail()
      {
         return false;
      }

      @Override
      SpatialNode createDesiredNode()
      {
         return null;
      }

      @Override
      void updateProgress(SpatialNode newNode)
      {
         lastStepProgress = minimumAllowableShortcut / pathDiff;
         if (lastStepProgress > 1.0)
            lastStepProgress = 1.0;
      }
   }
}