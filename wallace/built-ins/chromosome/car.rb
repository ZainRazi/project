# encoding: utf-8
import('../type/chromosome')


define_type(:car) do
  extends(:chromosome)
  composer do
    s(:chromosome, :car)
  end
end

define_specification(:car) do
  extends(:chromosome)

  ruby_file('fileutils')


  attribute :name,  ruby(:string)
  attribute :header,  ruby(:string)

  constructor [

                  parameter(:path,  ruby(:string)),

                  parameter(:body,  ruby(:string))

              ] do

    self.name = "DumbCarImpl"

    robots_path = "#{path}/robots/sample/evolved"



###################################
    header = File.read("UGV/dumbCarHeader.txt")
    body = "{header}


    {body}"
####################################


    x = "#{self}"
    x.slice!("#<Chromosome::Car:")
    x.slice!(">")


    # Write the code to a Java file in the evolved robots directory.
    #File.write("#{robots_path}/#{x}/#{name}.java", body)
    #FileUtils.rm_rf("#{Dir.home}/UGV/evolved/.", secure: true)
    FileUtils.mkdir_p("UGV/evolved/#{x}")
    File.write("UGV/evolved/#{x}/#{name}.java", body)

    ###this prints out the object name, could get it to save it in that folder, then get UGV to read that folder
    #print("!!")
    #print(self)

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

define_type(:car) do
  extends(:chromosome)
  composer do
    s(:chromosome, :car)
  end
end
