# encoding: utf-8
import('../type/chromosome')

define_type(:robocode) do
  extends(:chromosome)
  composer do
    s(:chromosome, :robocode)
  end
end

define_specification(:robocode) do
  extends(:chromosome)

  ruby_file('fileutils')

  # The name of this particular combatant.
  attribute :name,  ruby(:string)

  # Constructs a new robocode chromosome.
  constructor [

    # The path to the robocode directory.
    parameter(:path,  ruby(:string)),

    # The body of this robocode chromosome.
    parameter(:body,  ruby(:string))

  ] do

      # Produce (and store) a unique name for this tank.
      self.name = "Tank_#{self.object_id}"

      # Calculate the path to the evolved robots directory.
      robots_path = "#{path}/robots/sample/evolved"

      # Add the header to the body.
      body = "
      package sample.evolved;
      import robocode.*;
      public class #{name} extends AdvancedRobot {
      #{body}
      }"

      # Write the code to a Java file in the evolved robots directory.
      File.write("#{robots_path}/#{name}.java", body)

      # Write the properties file for the robot.
      File.write("#{robots_path}/#{name}.properties", "
      #Robot Properties
      robot.description=Evolved using Wallace
      robot.webpage=
      robocode.version=1.1.2
      robot.java.source.included=true
      robot.author.name=Wallace
      robot.classname=sample.evolved.#{name}
      robot.name=#{name}")

      # Compile the Java code to a class file.
      #system("javac -classpath #{path}/libs/robocode.jar #{robots_path}/#{name}.java")
    
  end

  method :unlink, accepts: [
    parameter(:path,  ruby(:string))
  ] do
    File.delete("#{path}/robots/sample/evolved/#{name}.java")
    File.delete("#{path}/robots/sample/evolved/#{name}.class")
    File.delete("#{path}/robots/sample/evolved/#{name}.properties")
  end

  # Returns the full name of this robocode controller, including its namespaces.
  method :full_name, returns: ruby(:string) do
    "sample.evolved.#{self.name}"
  end

end

define_type(:robocode) do
  extends(:chromosome)
  composer do
    s(:chromosome, :robocode)
  end
end
