package us.ihmc.avatar.joystickBasedJavaFXController;

import static us.ihmc.avatar.joystickBasedJavaFXController.GraspingJavaFXTopics.SphereRadius;
import static us.ihmc.avatar.joystickBasedJavaFXController.GraspingJavaFXTopics.SphereRequestCreate;
import static us.ihmc.avatar.joystickBasedJavaFXController.GraspingJavaFXTopics.SphereRequestClear;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Slider;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class GraspingObjectPaneController
{
   private JavaFXMessager messager;

   @FXML
   private Button btnCreateSphere;

   @FXML
   private Button btnClearSphere;

   @FXML
   private Slider sliderSphereRadius;

   public GraspingObjectPaneController()
   {

   }

   public void initialize(JavaFXMessager messager)
   {
      this.messager = messager;

      sliderSphereRadius.setValue(0.1);
      messager.bindBidirectional(SphereRadius, sliderSphereRadius.valueProperty(), createConverter(), true);
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
      System.out.println("Radius is " + sliderSphereRadius.valueProperty().doubleValue());

      messager.submitMessage(SphereRequestCreate, true);
   }

   @FXML
   private void clearSphere()
   {
      System.out.println("Clear Sphere");
      
      messager.submitMessage(SphereRequestClear, true);
   }
}
