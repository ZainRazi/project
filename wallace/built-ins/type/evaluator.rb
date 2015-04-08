# encoding: utf-8
import('bootstrap')
import('rng')
import('statistics')
import('population')
import('termination')

# Evaluators are responsible for calculating the fitness values for each
# member of a population.
define_specification :evaluator do

  # Evaluates the fitness of all individuals within a provided population.
  abstract_method :evaluate, returns: ruby(:integer), accepts: [
    parameter(:rng,         s(:rng)),
    parameter(:statistics,  s(:statistics)),
    parameter(:population,  s(:population)),
    parameter(:termination, s(:termination))
  ]

end

define_type :evaluator do
  composer { s(:evaluator) }
end
