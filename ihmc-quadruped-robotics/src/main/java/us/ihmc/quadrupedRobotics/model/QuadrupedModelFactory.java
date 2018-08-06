package us.ihmc.quadrupedRobotics.model;

import java.io.InputStream;
import java.util.Collection;

import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotDescription.RobotDescription;

public abstract class QuadrupedModelFactory implements FullQuadrupedRobotModelFactory
{
   public abstract RobotDescription createSdfRobot();

   @Override
   public abstract FullQuadrupedRobotModel createFullRobotModel();

   public abstract Collection<QuadrupedJointName> getQuadrupedJointNames();

   public abstract String getSDFNameForJointName(QuadrupedJointName quadrupedJointName);

   public abstract String getParameterResourceName(QuadrupedControlMode controlMode);

   public abstract InputStream getParameterInputStream(QuadrupedControlMode controlMode);
}
