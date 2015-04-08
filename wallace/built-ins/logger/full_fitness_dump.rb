# encoding: utf-8
import('csv')

define_type :full_fitness_dump do
  extends(:logger, :csv)
  composer { s(:logger, :full_fitness_dump) }
end

define_specification :full_fitness_dump do
  extends(:logger, :csv)

  # Constructs a new logger.
  constructor [
    parameter(:output_directory, ruby(:string))
  ] do
    FileUtils::mkdir_p(output_directory)
    super("#{output_directory}/full_fitness.csv", ["Generations"])
  end

  method :log, accepts: [
    parameter(:rng,         s(:rng)),
    parameter(:statistics,  s(:statistics)),
    parameter(:population,  s(:population))
  ] do

    # Create a new row for this generation.
    write_row()

    # Write the number of generations passed to the first cell.
    write_cell(statistics.iterations)

    # Write the fitness of each individual to a cell thereafter.
    population.demes.each do |deme|
      deme.contents.each do |ind|

        # Check that the individual has actually been evaluted!
        if ind.evaluated
          write_cell(ind.fitness.class == Fitness::Worst ? '' : ind.fitness.to_s)
        end
        
      end
    end

  end

end
