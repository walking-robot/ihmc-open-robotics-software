package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

/**
 * What to do this.
 * Handler grasping object.
 * Communicate with WholeBodyTrajectoryToolbox.
 * Control fingers.
 * Visualize object and preview of motion.
 */

public class GraspingJavaFXController
{
   private final AtomicReference<List<Node>> objectsToVisualizeReference = new AtomicReference<>(null);
   
   private final AtomicReference<Double> sphereRadius;
   
   private final AnimationTimer animationTimer;

   
   private final Group rootNode = new Group();
   
   public GraspingJavaFXController(JavaFXMessager messager)
   {
      
      
      sphereRadius = messager.createInput(GraspingJavaFXTopics.SphereRadius, 0.1);
      
      
      
      
      
      
      
      
      animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long arg0)
         {
            update();
         }
      };
      
      
      
      
   }
   
   private void update()
   {

   }
   
   public void start()
   {
      animationTimer.start();
      
   }

   public void stop()
   {
      animationTimer.stop();
   }
   
   public Node getRootNode()
   {
      return rootNode;
   }
}
