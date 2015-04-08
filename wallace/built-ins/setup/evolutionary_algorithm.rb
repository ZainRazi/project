# encoding: utf-8
import('../type/bootstrap')
import('../type/setup')
import('../type/population')
import('../type/termination')
import('../type/statistics')
import('../type/rng')
import('../type/breeder')
import('../type/replacer')
import('../type/evaluator')
import('../type/logger')

define_type :evolutionary_algorithm do
  extends(:setup)
  composer do |opts|

    # Construct the specification.
    s = specification(:setup, :evolutionary_algorithm).extend(name)

    # Compose each of the components.
    s_rng           = opts[:rng].compose
    s_statistics    = opts[:statistics].compose
    s_individual    = opts[:individual].compose
    s_population    = opts[:population].compose(s_individual)
    s_breeder       = opts[:breeder].compose
    si_replacer     = opts[:replacer].compose
    s_evaluator     = opts[:evaluator].compose
    s_termination   = opts[:termination].compose(
      statistics: s_statistics,
      population: s_population
    )

    # Compose each logger and inject it as a dependency.
    opts[:loggers].map! { |l|
      l = l.compose
      s.dependency(l.specification)
      l
    }

    # Refine the specification.
    s.attribute(:rng,         s_rng)
    s.attribute(:population,  s_population)
    s.attribute(:termination, s_termination)
    s.attribute(:statistics,  s_statistics)
    s.attribute(:breeder,     s_breeder)
    s.attribute(:replacer,    si_replacer.specification)
    s.attribute(:evaluator,   s_evaluator)
    s.attribute(:loggers,     s.list(s.s(:logger)))
    s.constructor([], source: "
      self.rng = #{s_rng.instance}
      self.population = #{s_population.instance}
      self.termination = #{s_termination.instance}
      self.statistics = #{s_statistics.instance}
      self.breeder = #{s_breeder.instance}
      self.replacer = #{si_replacer}
      self.evaluator = #{s_evaluator.instance}
      self.loggers = [#{opts[:loggers].map { |l| l.to_s }.join(', ')}]
    ")
    s

  end
end

define_specification :evolutionary_algorithm do
  extends(:setup)

  attribute :population,  s(:population)
  attribute :termination, s(:termination)
  attribute :statistics,  s(:statistics)
  attribute :rng,         s(:rng)
  attribute :breeder,     s(:breeder)
  attribute :replacer,    s(:replacer)
  attribute :evaluator,   s(:evaluator)
  attribute :logger,      s(:logger)

  method :debug do
    puts
    puts "Iteration: #{statistics.iterations}"
    puts "Evaluations: #{statistics.evaluations}"
    puts "Best Fitness: #{statistics.best_fitness.to_s}"

    #add saving genotype here

    file = File.open("/Users/zain/genotype/geno", "w")
    file.write("#{statistics.best_individual.genotype}")

    file.close()

  end

  method :evolve do

    # Generate the initial population at random, before evaluating it.
    population.generate(rng)
    evaluator.evaluate(rng, statistics, population, termination)

    # Find the best individual from the population and store it as the best
    # found by the search so far. This feels as though it should be outside
    # of statistics.
    statistics.best_individual = population.best
    statistics.best_fitness = statistics.best_individual.fitness

    # DEBUGGING
    debug()

    # Prepare and call each logger.
    loggers.each do |logger|
      logger.prepare()
      logger.call(rng, statistics, population)
    end

    # Keep on repeating the evolutionary cycle until any of the termination
    # conditions of the algorithm are satisfied.
    until termination.satisfied?(rng, statistics, population)

      statistics.iterations = statistics.iterations + 1

      # === Perform migration ===

      population.demes.each do |deme|
        deme.offspring = breeder.breed(rng, statistics, deme)
      end
      evaluator.evaluate(rng, statistics, population, termination)
      replacer.replace(rng, statistics, population)

      # Update the best individual and raw fitness if applicable.
      # This should be done by selecting the best offspring, not the best
      # member of the population after replacement. Must be done outside of
      # any parallel sections.
      p_best = population.best
      if !p_best.nil? && p_best > statistics.best_individual
        statistics.best_individual = p_best
        statistics.best_fitness = p_best.fitness
      end

      # DEBUGGING
      debug()

      # Call each logger.
      loggers.each do |logger|
        logger.call(rng, statistics, population)
      end

    end

    # Close each logger.
    loggers.each do |logger|
      logger.close()
    end

    # Return the final search statistics.
    return statistics
    
  end

end