package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.awt.image.BufferedImage;

public class SimpleVideoForwardingBehavior extends ImageProcessingBehavior
{
   public SimpleVideoForwardingBehavior(BehaviorCommunicationBridge communicationBridge, PacketDestination packetForwardDestination)
   {
      super("SimplevVideoForwarder", communicationBridge, packetForwardDestination);
   }

   @Override
   public void processImageToSend(BufferedImage bufferedImageToPack, Point3d cameraPositionToPack, Quat4d cameraOrientationToPack,
         IntrinsicParameters intrinsicParametersToPack)
   {
   }


   

   @Override
   public boolean isDone()
   {
      return false;
   }



   @Override
   public void initialize()
   {
      for(ConcurrentListeningQueue<VideoPacket> queue : listeningNetworkProcessorQueues.get(VideoPacket.class))
      {
         queue.clear();
      }
   }

  
}
