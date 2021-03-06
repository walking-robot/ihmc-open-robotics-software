package us.ihmc.simulationToolkit.outputWriters;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class PerfectPositionControlSimulatedOutputWriter implements OutputWriter
{
   private final String name;
   protected final FloatingRootJointRobot robot;
   protected ImmutablePair<FloatingJoint, FloatingInverseDynamicsJoint> rootJointPair;
   protected final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>>();
   private final JointDesiredOutputListReadOnly jointDesiredOutputListReadOnly;

   public PerfectPositionControlSimulatedOutputWriter(FloatingRootJointRobot robot, FullRobotModel fullRobotModel,
                                                      JointDesiredOutputListReadOnly jointDesiredOutputList)
   {
      this.name = robot.getName() + "SimulatedSensorReader";
      this.robot = robot;
      this.jointDesiredOutputListReadOnly = jointDesiredOutputList;

      setFullRobotModel(fullRobotModel);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      revoluteJoints.clear();
      OneDoFJoint[] revoluteJointsArray = fullRobotModel.getOneDoFJoints();

      for (OneDoFJoint revoluteJoint : revoluteJointsArray)
      {
         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);

         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDoFJoint, revoluteJoint);
         this.revoluteJoints.add(jointPair);
      }

      rootJointPair = new ImmutablePair<FloatingJoint, FloatingInverseDynamicsJoint>(robot.getRootJoint(), fullRobotModel.getRootJoint());
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   @Override
   public void write()
   {
      for (int i = 0; i < revoluteJoints.size(); i++)
      {
         ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = revoluteJoints.get(i);
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

         JointDesiredOutputReadOnly jointOutput = jointDesiredOutputListReadOnly.getJointDesiredOutput(revoluteJoint);

         if (jointOutput.hasDesiredPosition())
            pinJoint.setQ(jointOutput.getDesiredPosition());
         if (jointOutput.hasDesiredVelocity())
            pinJoint.setQd(jointOutput.getDesiredVelocity());
      }
   }
}
