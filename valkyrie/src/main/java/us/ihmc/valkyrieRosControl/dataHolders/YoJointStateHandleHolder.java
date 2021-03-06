package us.ihmc.valkyrieRosControl.dataHolders;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.rosControl.wholeRobot.JointStateHandle;

/**
 * Created by parallels on 7/6/16.
 */
public class YoJointStateHandleHolder
{
   private final String name;
   private final JointStateHandle handle;
   private final OneDoFJoint joint;

   private final YoDouble tauMeasured;
   private final YoDouble q;
   private final YoDouble qd;

   public YoJointStateHandleHolder(JointStateHandle handle, OneDoFJoint joint, YoVariableRegistry parentRegistry)
   {
      this.name = handle.getName();
      YoVariableRegistry registry = new YoVariableRegistry(name);

      this.handle = handle;
      this.joint = joint;

      this.tauMeasured = new YoDouble(name + "TauMeasured", registry);
      this.q = new YoDouble(name + "_q", registry);
      this.qd = new YoDouble(name + "_qd", registry);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      this.q.set(handle.getPosition());
      this.qd.set(handle.getVelocity());
      this.tauMeasured.set(handle.getEffort());
   }

   public OneDoFJoint getOneDoFJoint()
   {
      return joint;
   }

   public double getTauMeasured()
   {
      return tauMeasured.getDoubleValue();
   }

   public double getQ()
   {
      return q.getDoubleValue();
   }

   public double getQd()
   {
      return qd.getDoubleValue();
   }

   public String getName()
   {
      return this.name;
   }
}
