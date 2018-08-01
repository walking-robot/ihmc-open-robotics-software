package us.ihmc.avatar.joystickBasedJavaFXController;

import static us.ihmc.avatar.joystickBasedJavaFXController.GraspingJavaFXTopics.SphereRadius;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Slider;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class GraspingObjectPaneController
{
   @FXML
   private Button btnCreateSphere;

   @FXML
   private Button btnClearSphere;

   @FXML
   private Slider sliderSphereRadius;

   private boolean isSphereCreated = false;

   public GraspingObjectPaneController()
   {

   }

   public void initialize(JavaFXMessager messager)
   {
      sliderSphereRadius.setValue(0.1);
      PrintTools.info("");
      messager.bindBidirectional(SphereRadius, sliderSphereRadius.valueProperty(), createConverter(), true);
      PrintTools.info("");
   }

   private PropertyToMessageTypeConverter<Double, Number> createConverter()
   {
      return new PropertyToMessageTypeConverter<Double, Number>()
      {
         @Override
         public Double convert(Number propertyValue)
         {
            return propertyValue.doubleValue();
         }

         @Override
         public Number interpret(Double messageContent)
         {
            return messageContent;
         }
      };
   }

   @FXML
   private void createSphere()
   {
      System.out.println("Create Sphere");
      System.out.println("Radius is " + sliderSphereRadius.valueProperty().doubleValue() + " " + isSphereCreated());
      isSphereCreated = true;
   }

   @FXML
   private void clearSphere()
   {
      System.out.println("Clear Sphere" + " " + isSphereCreated());
      isSphereCreated = false;
   }

   public boolean isSphereCreated()
   {
      return isSphereCreated;
   }
}
