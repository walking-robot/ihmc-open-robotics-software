package us.ihmc.footstepPlanning.graphSearch.planners;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.GraphVisualization;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlan;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.YoVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

public class VisibilityGraphWithAStarPlanner implements FootstepPlanner
{
   private static final boolean DEBUG = true;
   private static final double defaultHeuristicWeight = 2.0;
   private static final double defaultPlanningHorizon = 1.0;
   private static final double defaultTimeout = 5.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble timeout = new YoDouble("timeout", registry);

   private final YoDouble timeSpentBeforeFootstepPlanner = new YoDouble("timeSpentBeforeFootstepPlanner", registry);
   private final YoDouble timeSpentInFootstepPlanner = new YoDouble("timeSpentInFootstepPlanner", registry);
   private final YoEnum<FootstepPlanningResult> yoResult = new YoEnum<>("planningResult", registry, FootstepPlanningResult.class);
   private final NavigableRegionsManager navigableRegionsManager;

   private final YoDouble planningHorizon = new YoDouble("planningHorizon", registry);

   private final FootstepPlannerParameters parameters;
   private final WaypointDefinedBodyPathPlan bodyPath;
   private final BodyPathHeuristics heuristics;
   private final FootstepPlanner footstepPlanner;

   private PlanarRegionsList planarRegionsList;
   private final FramePose3D bodyStartPose = new FramePose3D();
   private final FramePose3D bodyGoalPose = new FramePose3D();
   private final List<Point2D> waypoints = new ArrayList<>();

   private final boolean visualizing;
   private static final int bodyPathPointsForVisualization = 500;
   private final List<YoFramePoint3D> bodyPathPoints = new ArrayList<>();
   private YoFramePoint3D goalPosition;


   public VisibilityGraphWithAStarPlanner(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons,
                                          YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(parameters, new SnapBasedNodeChecker(parameters, footPolygons, new SimplePlanarRegionFootstepNodeSnapper(footPolygons)),
           new ParameterBasedNodeExpansion(parameters), new DistanceAndYawBasedCost(parameters), new FootstepNodeSnapAndWiggler(footPolygons, parameters, null),
           null, graphicsListRegistry, parentRegistry);
   }

   public VisibilityGraphWithAStarPlanner(FootstepPlannerParameters parameters, FootstepNodeChecker nodeChecker, FootstepNodeExpansion nodeExpansion,
                                          FootstepCost stepCostCalculator, FootstepNodeSnapper snapper, GraphVisualization viz,
                                          YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.parameters = parameters;
      bodyPath = new WaypointDefinedBodyPathPlan();
      heuristics = new BodyPathHeuristics(registry, parameters, bodyPath);

      planningHorizon.set(defaultPlanningHorizon);

      heuristics.setWeight(defaultHeuristicWeight);
      footstepPlanner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, nodeExpansion, stepCostCalculator, snapper, viz, registry);

      this.navigableRegionsManager = new NavigableRegionsManager(new YoVisibilityGraphParameters(new DefaultVisibilityGraphParameters(), registry));

      timeout.set(defaultTimeout);
      visualizing = graphicsListRegistry != null;
      if (visualizing)
      {
         setupVisualization(graphicsListRegistry, registry);
      }
   }

   public void setHeuristicWeight(double heuristicWeight)
   {
      heuristics.setWeight(heuristicWeight);
   }

   public void setPlanningHorizon(double planningHorizon)
   {
      this.planningHorizon.set(planningHorizon);
   }

   private void setupVisualization(YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         YoFramePoint3D point = new YoFramePoint3D("BodyPathPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         point.setToNaN();
         bodyPathPoints.add(point);
         YoGraphicPosition pointVisualization = new YoGraphicPosition("BodyPathPoint" + i, point, 0.02, YoAppearance.Yellow());
         graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), pointVisualization);
      }


      goalPosition = new YoFramePoint3D("GoalPosition", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition pointVisualization = new YoGraphicPosition("GoalPosition", goalPosition, 0.05, YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), pointVisualization);
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      double defaultStepWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);
      bodyStartPose.setToZero(stanceFrame);
      bodyStartPose.setY(side.negateIfLeftSide(defaultStepWidth / 2.0));
      bodyStartPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepPlanner.setInitialStanceFoot(stanceFootPose, side);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      AStarFootstepPlanner.checkGoalType(goal);
      bodyGoalPose.setIncludingFrame(goal.getGoalPoseBetweenFeet());
   }

   @Override
   public void setTimeout(double timeout)
   {
      this.timeout.set(timeout);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      footstepPlanner.setPlanarRegions(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      long startTime = System.currentTimeMillis();
      waypoints.clear();

      if (planarRegionsList == null)
      {
         waypoints.add(new Point2D(bodyStartPose.getPosition()));
         waypoints.add(new Point2D(bodyGoalPose.getPosition()));
      }
      else
      {
         Point3D startPos = PlanarRegionTools.projectPointToPlanesVertically(bodyStartPose.getPosition(), planarRegionsList);
         Point3D goalPos = PlanarRegionTools.projectPointToPlanesVertically(bodyGoalPose.getPosition(), planarRegionsList);
         navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

         if (startPos == null)
         {
            PrintTools.info("adding plane at start foot");
            startPos = new Point3D(bodyStartPose.getX(), bodyStartPose.getY(), 0.0);
            addPlanarRegionAtZeroHeight(bodyStartPose.getX(), bodyStartPose.getY());
         }
         if (goalPos == null)
         {
            PrintTools.info("adding plane at goal pose");
            goalPos = new Point3D(bodyGoalPose.getX(), bodyGoalPose.getY(), 0.0);
            addPlanarRegionAtZeroHeight(bodyGoalPose.getX(), bodyGoalPose.getY());
         }

         if (DEBUG)
         {
            PrintTools.info("Starting to plan using )" + getClass().getSimpleName());
            PrintTools.info("Body start pose: " + startPos);
            PrintTools.info("Body goal pose:  " + goalPos);
         }

         try
         {
            List<Point3DReadOnly> path = new ArrayList<>(navigableRegionsManager.calculateBodyPath(startPos, goalPos));

            if (path.size() < 2)
            {
               if (parameters.getReturnBestEffortPlan())
               {
                  Vector2D goalDirection = new Vector2D(bodyGoalPose.getPosition());
                  goalDirection.sub(bodyStartPose.getX(), bodyStartPose.getY());
                  goalDirection.scale(planningHorizon.getDoubleValue() / goalDirection.length());
                  waypoints.add(new Point2D(goalDirection.getX() + bodyStartPose.getX(), goalDirection.getY() + bodyStartPose.getY()));
               }
               else
               {
                  double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
                  timeSpentBeforeFootstepPlanner.set(seconds);
                  timeSpentInFootstepPlanner.set(0.0);
                  yoResult.set(FootstepPlanningResult.PLANNER_FAILED);
                  return yoResult.getEnumValue();
               }
            }

            for (Point3DReadOnly waypoint3d : path)
            {
               waypoints.add(new Point2D(waypoint3d.getX(), waypoint3d.getY()));
            }
         }
         catch (Exception e)
         {
            e.printStackTrace();
            double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
            timeSpentBeforeFootstepPlanner.set(seconds);
            timeSpentInFootstepPlanner.set(0.0);
            yoResult.set(FootstepPlanningResult.PLANNER_FAILED);
            return yoResult.getEnumValue();
         }
      }

      bodyPath.setWaypoints(waypoints);
      bodyPath.compute(null, null);



      Pose2D goalPose2d = new Pose2D();
      double pathLength = bodyPath.computePathLength(0.0);
            double alpha = MathTools.clamp(planningHorizon.getValue() / pathLength, 0.0, 1.0);
//      double alpha = 1.0;
      bodyPath.getPointAlongPath(alpha, goalPose2d);
      heuristics.setGoalAlpha(alpha);

      if (visualizing)
      {
         updateBodyPathVisualization(goalPose2d);
      }

      FramePose3D footstepPlannerGoal = new FramePose3D();
      footstepPlannerGoal.setPosition(goalPose2d.getX(), goalPose2d.getY(), 0.0);
      footstepPlannerGoal.setOrientationYawPitchRoll(goalPose2d.getYaw(), 0.0, 0.0);

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(footstepPlannerGoal);
      footstepPlanner.setGoal(goal);

      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentBeforeFootstepPlanner.set(seconds);
      footstepPlanner.setTimeout(timeout.getDoubleValue() - seconds);

      startTime = System.currentTimeMillis();
      yoResult.set(footstepPlanner.plan());
      seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentInFootstepPlanner.set(seconds);

      if (DEBUG)
      {
         PrintTools.info("Visibility graph with A* planner finished. Result: " + yoResult.getEnumValue());
      }

      return yoResult.getEnumValue();
   }

   // TODO hack to add start and goal planar regions
   private void addPlanarRegionAtZeroHeight(double xLocation, double yLocation)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.3, 0.3);
      polygon.addVertex(-0.3, 0.3);
      polygon.addVertex(0.3, -0.3);
      polygon.addVertex(-0.3, -0.25);
      polygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(new AxisAngle(), new Vector3D(xLocation, yLocation, 0.0)), polygon);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   private void updateBodyPathVisualization(Pose2D goalPose)
   {
      Pose2D tempPose = new Pose2D();
      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         double percent = (double) i / (double) (bodyPathPointsForVisualization - 1);
         bodyPath.getPointAlongPath(percent, tempPose);
         if (planarRegionsList == null)
         {
            bodyPathPoints.get(i).set(tempPose.getPosition(), 1.0);
         }
         else
         {
            Point3D position = new Point3D();
            position.set(tempPose.getPosition());
            Point3D projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(position, planarRegionsList);
            if (projectedPoint != null)
            {
               bodyPathPoints.get(i).set(projectedPoint);
            }
            else
            {
               bodyPathPoints.get(i).setToNaN();
            }
         }
      }

      if (planarRegionsList == null)
      {
         goalPosition.set(goalPose.getPosition(), 1.0);
      }
      else
      {
         Point3D position = new Point3D();
         position.set(goalPose.getPosition());
         Point3D projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(position, planarRegionsList);
         if (projectedPoint != null)
         {
            goalPosition.set(projectedPoint);
         }
         else
         {
            goalPosition.setToNaN();
         }
      }
   }

   public Point3D[][] getNavigableRegions()
   {
      return navigableRegionsManager.getNavigableExtrusions();
   }

   public List<Point2D> getBodyPathWaypoints()
   {
      return waypoints;
   }

   public Pose2D getLowLevelPlannerGoal()
   {
      Pose2D goalPose2d = new Pose2D();
      double pathLength = bodyPath.computePathLength(0.0);
      double alpha = MathTools.clamp(planningHorizon.getDoubleValue() / pathLength, 0.0, 1.0);
      bodyPath.getPointAlongPath(alpha, goalPose2d);
      return goalPose2d;
   }

   public BodyPathPlanner getBodyPathPlanner()
   {
      return bodyPath;
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlanner.getPlan();
   }

   public static VisibilityGraphWithAStarPlanner createFlatGroundPlanner(FootstepPlannerParameters parameters, GraphVisualization viz,
                                                                         YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      AlwaysValidNodeChecker nodeChecker = new AlwaysValidNodeChecker();
      FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();

      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      DistanceAndYawBasedCost stepCostCalculator = new DistanceAndYawBasedCost(parameters);

      VisibilityGraphWithAStarPlanner planner = new VisibilityGraphWithAStarPlanner(parameters, nodeChecker, expansion, stepCostCalculator, snapper, viz, graphicsListRegistry, registry);
      planner.setHeuristicWeight(2.0);


      return planner;
   }

   public static VisibilityGraphWithAStarPlanner createRoughTerrainPlanner(FootstepPlannerParameters parameters,
                                                                           SideDependentList<ConvexPolygon2D> footPolygons, GraphVisualization viz,
                                                                           YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);

//      FootstepCost footstepCost = new DistanceAndYawBasedCost(parameters);
      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setUseQuadraticStepCost(true);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setUsePitchAndRollCost(true);

      FootstepCost footstepCost = costBuilder.buildCost();



      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      FootstepNodeSnapper postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters, null);

      VisibilityGraphWithAStarPlanner planner = new VisibilityGraphWithAStarPlanner(parameters, nodeChecker, expansion, footstepCost, postProcessingSnapper, viz, graphicsListRegistry,
                                                 registry);
      planner.setHeuristicWeight(2.0);

      return planner;
   }

}
