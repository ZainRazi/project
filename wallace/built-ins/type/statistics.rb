# encoding: utf-8
import('bootstrap')
import('statistic')

define_specification :statistics do

  # For now!
  attribute :evaluations,       ruby(:integer)
  attribute :iterations,        ruby(:integer)
  attribute :best_individual,   s(:individual)
  attribute :best_fitness,      s(:fitness)

  constructor [] do
    self.evaluations = 0
    self.iterations = 0
    self.best_individual = nil
    self.best_fitness = nil
  end

end

define_type :statistics do

  #indexed_collection :statistics, :statistics, i(:statistic)

  composer do
    s = spec(:statistics)

    # Add each of the statistics to the statistics specification.

    s
  end

end
