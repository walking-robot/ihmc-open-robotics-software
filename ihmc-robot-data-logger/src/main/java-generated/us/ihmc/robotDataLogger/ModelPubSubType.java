package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.idl.CDR;
import us.ihmc.idl.IDLSequence;

/**
* 
* Topic data type of the struct "Model" defined in "LogProperties.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LogProperties.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LogProperties.idl instead.
*
*/
public class ModelPubSubType implements TopicDataType<us.ihmc.robotDataLogger.Model>
{
	public static final String name = "us::ihmc::robotDataLogger::Model";
	
	
	
    public ModelPubSubType()
    {
        
    }

	private final CDR serializeCDR = new CDR();
	private final CDR deserializeCDR = new CDR();

    
    @Override
   public void serialize(us.ihmc.robotDataLogger.Model data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }
   @Override
   public void deserialize(SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Model data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }
   
	public static int getMaxCdrSerializedSize()
	{
		return getMaxCdrSerializedSize(0);
	}

	public static int getMaxCdrSerializedSize(int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < 255; ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + 255 + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}


	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Model data)
	{
		return getCdrSerializedSize(data, 0);
	}

	public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Model data, int current_alignment)
	{
	    int initial_alignment = current_alignment;
	            
	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getLoader().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getPath().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getResourceBundle().length() + 1;

	    current_alignment += 4 + CDR.alignment(current_alignment, 4);
	    for(int a = 0; a < data.getResourceDirectoriesList().size(); ++a)
	    {
	        current_alignment += 4 + CDR.alignment(current_alignment, 4) + data.getResourceDirectoriesList().get(a).length() + 1;
	    }
	
	    return current_alignment - initial_alignment;
	}
	
   public static void write(us.ihmc.robotDataLogger.Model data, CDR cdr)
   {

	    if(data.getLoader().length() <= 255)
	    cdr.write_type_d(data.getLoader());else
	        throw new RuntimeException("loader field exceeds the maximum length");

	    if(data.getPath().length() <= 255)
	    cdr.write_type_d(data.getPath());else
	        throw new RuntimeException("path field exceeds the maximum length");

	    if(data.getName().length() <= 255)
	    cdr.write_type_d(data.getName());else
	        throw new RuntimeException("name field exceeds the maximum length");

	    if(data.getResourceBundle().length() <= 255)
	    cdr.write_type_d(data.getResourceBundle());else
	        throw new RuntimeException("resourceBundle field exceeds the maximum length");

	    if(data.getResourceDirectoriesList().size() <= 255)
	    cdr.write_type_e(data.getResourceDirectoriesList());else
	        throw new RuntimeException("resourceDirectoriesList field exceeds the maximum length");
   }

   public static void read(us.ihmc.robotDataLogger.Model data, CDR cdr)
   {

	    	cdr.read_type_d(data.getLoader());	

	    	cdr.read_type_d(data.getPath());	

	    	cdr.read_type_d(data.getName());	

	    	cdr.read_type_d(data.getResourceBundle());	

	    	cdr.read_type_e(data.getResourceDirectoriesList());	
   }
   
	@Override
	public final void serialize(us.ihmc.robotDataLogger.Model data, InterchangeSerializer ser)
	{
			    ser.write_type_d("loader", data.getLoader());
			    
			    ser.write_type_d("path", data.getPath());
			    
			    ser.write_type_d("name", data.getName());
			    
			    ser.write_type_d("resourceBundle", data.getResourceBundle());
			    
			    ser.write_type_e("resourceDirectoriesList", data.getResourceDirectoriesList());
			    
	}
	
	@Override
	public final void deserialize(InterchangeSerializer ser, us.ihmc.robotDataLogger.Model data)
	{
	    			ser.read_type_d("loader", data.getLoader());	
	    	    
	    			ser.read_type_d("path", data.getPath());	
	    	    
	    			ser.read_type_d("name", data.getName());	
	    	    
	    			ser.read_type_d("resourceBundle", data.getResourceBundle());	
	    	    
	    			ser.read_type_e("resourceDirectoriesList", data.getResourceDirectoriesList());	
	    	    
	}

   public static void staticCopy(us.ihmc.robotDataLogger.Model src, us.ihmc.robotDataLogger.Model dest)
   {
      dest.set(src);
   }
   
   
   @Override
   public us.ihmc.robotDataLogger.Model createData()
   {
      return new us.ihmc.robotDataLogger.Model();
   }
      

   @Override
   public int getTypeSize()
   {
      return CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public String getName()
   {
      return name;
   }
   
   public void serialize(us.ihmc.robotDataLogger.Model data, CDR cdr)
	{
		write(data, cdr);
	}

   public void deserialize(us.ihmc.robotDataLogger.Model data, CDR cdr)
   {
        read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.Model src, us.ihmc.robotDataLogger.Model dest)
   {
      staticCopy(src, dest);
   }	

   
   @Override
   public ModelPubSubType newInstance()
   {
   	  return new ModelPubSubType();
   }
}