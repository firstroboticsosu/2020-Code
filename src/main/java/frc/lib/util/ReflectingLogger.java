package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.exceptions.DirectoryNotFoundException;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.text.SimpleDateFormat;
import java.util.*;

public class ReflectingLogger<T> {

    private static final String fileSeperator = ", ";

    private PrintWriter output = null;
    private Map<Field, T> classFieldMap = new LinkedHashMap<>();

    public ReflectingLogger(List<T> subsystemIOs) throws FileNotFoundException{
        this(subsystemIOs, getMount("robotdata"), false);
    }

    public ReflectingLogger(List<T> subsystemIOs, File loggingFile){
        this(subsystemIOs, loggingFile, false);
    }

    public ReflectingLogger(List<T> subsystemIOs, File loggingFile, Boolean allowRethrow) {
        //generate map of subsystem IO's and fields
        for(T subsystemIO : subsystemIOs){
            for(Field field : subsystemIO.getClass().getFields()) {
                classFieldMap.put(field, subsystemIO);
            }
        }

        //create file reference
        try{
            output = new PrintWriter(loggingFile.getAbsolutePath());
            
            // Write field names
            StringBuffer line = new StringBuffer();
            line.append("time");
            for (Map.Entry<Field, T> entry : classFieldMap.entrySet()) {
                line.append(fileSeperator);
                line.append(entry.getKey().getName());
            }

            //Write the first line of the file
            writeLine(line.toString());
        } catch (FileNotFoundException e){
            //allows the code not to boot if logging functionality cannot start
            if(allowRethrow) throw new RuntimeException(e.getCause());

            //otherwise kick out the stack trace for the error and stop the logger
            e.printStackTrace();
        }
    }

    public void update(List<T> subsystemIOs) {

        //no writer avaliable to update exit the update
        if(output.equals(null)) return;

        //generate map of subsystem IO's and fields
        for(T subsystemIO : subsystemIOs){
            for(Field field : subsystemIO.getClass().getFields()) {
                classFieldMap.put(field, subsystemIO);
            }
        }

        final StringBuffer line = new StringBuffer();

        //Append starting time
        line.append(Timer.getFPGATimestamp());

        //for all fields in map generate
        for (Map.Entry<Field, T> entry : classFieldMap.entrySet()) {
            //append separator
            line.append(fileSeperator);

            //Attempt to append subsystem IO value
            try {
                if (CSVWritable.class.isAssignableFrom(entry.getKey().getType())) {
                    line.append(((CSVWritable) entry.getKey().get(entry.getValue())).toCSV());
                } else {
                    line.append(entry.getKey().get(entry.getValue()).toString());
                }
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        writeLine(line.toString());
    }

    protected synchronized void writeLine(String line) {
        if (output != null) {
            output.println(line);
            output.flush();
        }
    }

    public static File getMount(String subsystemName) throws FileNotFoundException {
        //create base file reference looking for the media directory
        File media = new File("/media");
        if (!media.exists()) throw new DirectoryNotFoundException("/media");

        if(media.listFiles().length < 1) throw new DirectoryNotFoundException("No media devices found in system");

        //Locate the currently active media drive by finding a nested logging directory
        File logging_path = null;
        for (File mount : media.listFiles()) {
            logging_path = new File(mount.getAbsolutePath() + "/logging");
            if (logging_path.isDirectory()) {
                System.out.println(logging_path.getAbsolutePath());
                break;
            }
            logging_path = null;
        }

        if(logging_path.equals(null)) throw new DirectoryNotFoundException("No media device with a logging directory was found");

        File fileref = new File(logging_path.getAbsolutePath() + File.separator + getTimeStampedFileName(subsystemName));

        //if(!fileref.canWrite()) throw new FileInvalidException(fileref.getAbsolutePath() + " cannot be written to");

        return fileref;

    }

    private static String getTimeStampedFileName(String subsystemName){
        SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
        outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
        String newDateString = outputFormatter.format(new Date());
        return subsystemName + "_" + newDateString + "_LOG.csv";
    }

}
