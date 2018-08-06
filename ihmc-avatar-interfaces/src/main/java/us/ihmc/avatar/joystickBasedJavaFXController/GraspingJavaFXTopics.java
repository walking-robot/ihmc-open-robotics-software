package us.ihmc.avatar.joystickBasedJavaFXController;

import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Category;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.TypedTopicTheme;

public class GraspingJavaFXTopics
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("Grasping"));

   private static final CategoryTheme GraspingObject = apiFactory.createCategoryTheme("GraspingObject");
   private static final CategoryTheme WholeBodyTranectoryToolbox = apiFactory.createCategoryTheme("WholeBodyTranectoryToolbox");
   private static final CategoryTheme FingerControl = apiFactory.createCategoryTheme("FingerControl");

   private static final CategoryTheme Sphere = apiFactory.createCategoryTheme("Sphere");
   private static final CategoryTheme Cylinder = apiFactory.createCategoryTheme("Cylinder");
   private static final CategoryTheme Torus = apiFactory.createCategoryTheme("Torus");

   private static final TypedTopicTheme<Double> Radius = apiFactory.createTypedTopicTheme("Radius");
   private static final TypedTopicTheme<Double> Height = apiFactory.createTypedTopicTheme("Height");
   private static final TypedTopicTheme<Double> TubeRadius = apiFactory.createTypedTopicTheme("TubeRadius");

   private static final TypedTopicTheme<Integer> Selected = apiFactory.createTypedTopicTheme("Selected");

   public static final Topic<Double> SphereRadius = Root.child(GraspingObject).child(Sphere).topic(Radius);
   public static final Topic<Double> CylinderRadius = Root.child(GraspingObject).child(Cylinder).topic(Radius);
   public static final Topic<Double> CylinderHeight = Root.child(GraspingObject).child(Cylinder).topic(Height);
   public static final Topic<Double> TorusRadius = Root.child(GraspingObject).child(Torus).topic(Radius);
   public static final Topic<Double> TorusTubeRadius = Root.child(GraspingObject).child(Torus).topic(TubeRadius);

   public static final Topic<Integer> SelectedShape = Root.child(GraspingObject).topic(Selected);

   static
   {
      apiFactory.includeMessagerAPIs(XBoxOneJavaFXController.XBoxOneControllerAPI);
   }

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}