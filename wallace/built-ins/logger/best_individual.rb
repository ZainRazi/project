# encoding: utf-8
import('../type/logger')

define_type :best_individual do
  extends(:logger)
  composer { s(:logger, :best_individual) }
end

define_specification :best_individual do
  extends(:logger)

  # Constructs a new best individual logger.
  constructor [
    parameter(:output_directory, ruby(:string))
  ] do
    super("#{Pathname.new(output_directory).cleanpath.to_s}/best_individual/")
  end

  # Creates a sub-directory to store each of the best individuals in.
  method :prepare do

    # Should be able to delete the output directory and rebuild it, but
    # this seems to cause issues with permissions.
    FileUtils::rm_rf("#{output_directory}/.")
    FileUtils::mkdir_p(output_directory)
    
  end

  method :log, accepts: [
    parameter(:rng,         s(:rng)),
    parameter(:statistics,  s(:statistics)),
    parameter(:population,  s(:population))
  ] do
    File.open("#{output_directory}/#{statistics.iterations}.txt", "w") do |f|
      f.write(population.best.describe)
    end
  end
end
