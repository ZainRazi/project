# encoding: utf-8
import('evolutionary_algorithm')
import('../breeder/simple')
import('../replacer/generational')

define_type :simple do
  extends(:setup, :evolutionary_algorithm)

  attribute :population,      ruby(:integer)
  attribute :elites,          ruby(:integer)
  attribute :representation,  i(:representation)
  attribute :evaluator,       t(:evaluator)

  collection :loggers, :logger, i(:logger)

  indexed_collection :termination_conditions, :termination_condition, i(:criterion)
  indexed_collection :selectors, :selector, i(:selector)
  indexed_collection :variators, :variator, i(:variator)

  composer do

    # Construct the termination criteria.
    termination = t(:termination)
    termination_conditions.each_pair do |n, c|
      termination.criterion(n, c)
    end

    # Construct the breeder.
    breeder = t(:breeder, :simple)
    selectors.each_pair { |n, s| breeder.selectors.add(n, s) } 
    variators.each_pair { |n, v| breeder.variators.add(n, v) }

    # Construct the population.
    set(elites, 0) if elites.nil?

    # Construct the replacer.
    replacer = t(:replacer, :generational).instance(elitism: elites)

    # Construct the population.
    population_size = population
    population = t(:population)
    population.deme(capacity: population_size, num_offspring: population_size - elites)
    population.demes[0].type.species.set(:representation, representation)
    
    # Construct the setup.
    sup(
      breeder:      breeder,
      evaluator:    evaluator,
      replacer:     replacer,
      population:   population,
      termination:  termination,
      statistics:   t(:statistics),
      individual:   t(:individual),
      rng:          t(:rng, :simple),
      loggers:      loggers.values,
    )

  end

end
