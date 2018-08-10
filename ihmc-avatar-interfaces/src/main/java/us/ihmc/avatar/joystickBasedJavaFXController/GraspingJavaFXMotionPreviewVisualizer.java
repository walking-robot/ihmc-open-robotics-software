package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;
import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class GraspingJavaFXMotionPreviewVisualizer
{
   private final GraphicsRobot graphicsRobot;
   private final JavaFXGraphics3DNode rootNode;
   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJoint[] allJoints;
   private final int jointNameHash;
   private final AtomicReference<RigidBodyTransform> newRootJointPoseReference = new AtomicReference<>(null);
   private final AtomicReference<float[]> newJointConfigurationReference = new AtomicReference<>(null);

   private final AtomicBoolean enable = new AtomicBoolean(false);
   private double localTime = 0.0;
   private WholeBodyTrajectoryToolboxOutputStatus packetInProgress = null;

   private final double onetickTime = 0.1;
   
   private final AnimationTimer animationTimer;

   public GraspingJavaFXMotionPreviewVisualizer(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      RobotDescription robotDescription = fullRobotModelFactory.getRobotDescription();
      graphicsRobot = new GraphicsIDRobot(robotDescription.getName(), fullRobotModel.getElevator(), robotDescription);
      rootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
      rootNode.setMouseTransparent(true);
      addNodesRecursively(graphicsRobot.getRootNode(), rootNode);
      rootNode.update();

      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = (int) NameBasedHashCodeTools.computeArrayHashCode(allJoints);

      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if (!enable.get())
            {
               packetInProgress = null;
               localTime = 0;
               return;
            }
            
            if (packetInProgress != null)
            {
               double trajectoryTime = packetInProgress.getTrajectoryTimes().get(packetInProgress.getTrajectoryTimes().size() - 1);

               KinematicsToolboxOutputStatus toShowOutputStatus = findFrameFromTime(packetInProgress, localTime); //get close outputstatus based on inverseKinematicsSolution.trajectoryTimes.

               if (toShowOutputStatus == null)
               {
                  PrintTools.error(this, "Could not find frame for t = " + localTime + ", stopping preview.");
                  enable(false);
                  return;
               }

               visualizeFrame(toShowOutputStatus);

               localTime += onetickTime;

               // for loop
               if (localTime >= trajectoryTime)
               {
                  // disable(); // It has been more than 'delayBeforeDisabling' number of ticks that we haven't received any KinematicsToolboxOutputStatus, shutting down the visualizer.
                  localTime = 0.0;
               }
            }

            RigidBodyTransform newRootJointPose = newRootJointPoseReference.getAndSet(null);
            if (newRootJointPose != null)
               fullRobotModel.getRootJoint().setPositionAndRotation(newRootJointPose);

            float[] newJointConfiguration = newJointConfigurationReference.getAndSet(null);
            if (newJointConfiguration != null)
            {
               for (int i = 0; i < allJoints.length; i++)
                  allJoints[i].setQ(newJointConfiguration[i]);
            }
            fullRobotModel.getElevator().updateFramesRecursively();
            graphicsRobot.update();
            rootNode.update();
         }
      };
   }

   public void start()
   {
      animationTimer.start();
   }

   public void stop()
   {
      animationTimer.stop();
   }

   private void addNodesRecursively(Graphics3DNode graphics3dNode, JavaFXGraphics3DNode parentNode)
   {
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode);
      parentNode.addChild(node);
      graphics3dNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node));
   }

   public void enable(boolean value)
   {
      enable.set(value);
   }

   public void submitWholeBodyTrajectoryToolboxOutputStatus(WholeBodyTrajectoryToolboxOutputStatus outputStatus)
   {
      packetInProgress = outputStatus;
      PrintTools.info("planning_result_ "+packetInProgress.planning_result_);
      PrintTools.info("robot_configurations_ "+packetInProgress.robot_configurations_.size());
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   private KinematicsToolboxOutputStatus findFrameFromTime(WholeBodyTrajectoryToolboxOutputStatus outputStatus, double time)
   {
      if (time <= 0.0)
         return outputStatus.getRobotConfigurations().get(0);

      else if (time >= outputStatus.getTrajectoryTimes().get(outputStatus.getTrajectoryTimes().size() - 1))
         return outputStatus.getRobotConfigurations().get(outputStatus.getRobotConfigurations().size() - 1);
      else
      {
         double timeGap = 0.0;

         int indexOfFrame = 0;
         int numberOfTrajectoryTimes = outputStatus.getTrajectoryTimes().size();

         for (int i = 0; i < numberOfTrajectoryTimes; i++)
         {
            timeGap = time - outputStatus.getTrajectoryTimes().get(i);
            if (timeGap < 0)
            {
               indexOfFrame = i;
               break;
            }
         }

         KinematicsToolboxOutputStatus frameOne = outputStatus.getRobotConfigurations().get(indexOfFrame - 1);
         KinematicsToolboxOutputStatus frameTwo = outputStatus.getRobotConfigurations().get(indexOfFrame);

         double timeOne = outputStatus.getTrajectoryTimes().get(indexOfFrame - 1);
         double timeTwo = outputStatus.getTrajectoryTimes().get(indexOfFrame);

         double alpha = (time - timeOne) / (timeTwo - timeOne);

         return MessageTools.interpolateMessages(frameOne, frameTwo, alpha);
      }
   }

   private void visualizeFrame(KinematicsToolboxOutputStatus frame)
   {
      if (jointNameHash != frame.getJointNameHash())
         throw new RuntimeException("Hashes are different.");

      float[] joints = new float[allJoints.length];
      for (int i = 0; i < joints.length; i++)
      {
         joints[i] = frame.getDesiredJointAngles().get(i);
      }

      newRootJointPoseReference.set(new RigidBodyTransform(frame.getDesiredRootOrientation(), frame.getDesiredRootTranslation()));
      newJointConfigurationReference.set(joints);
   }
}
