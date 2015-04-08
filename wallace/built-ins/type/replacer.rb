# encoding: utf-8
import('bootstrap')
import('individual')
import('rng')
import('statistics')

# Replacers are used to determine the contents of a deme for the next
# generation from the members, parents and offspring of the current generation.
define_specification :replacer do

  # Performs replacement on a given population.
  method :replace, accepts: [
    parameter(:rng,         s(:rng)),
    parameter(:statistics,  s(:statistics)),
    parameter(:population,  s(:population))
  ] do
    population.demes.each do |deme|
      deme.contents = survivors(rng, statistics, deme.contents, deme.offspring)
    end
  end

  # Calculates the surviving individuals from the union of the multi-set of 
  # individuals already in the population, and the newly created offspring.
  #
  # ==== Parameters
  # [+random+]      The random number generator to use.
  # [+statistics+]  The current search statistics.
  # [+members+]     The existing members of the deme.
  # [+offspring+]   The newly created offspring for the deme.
  #
  # ==== Returns
  # The set of surviving individuals.
  abstract_method :survivors, returns: list(s(:individual)), accepts: [
    parameter(:rng,         s(:rng)),
    parameter(:statistics,  s(:statistics)),
    parameter(:members,     list(s(:individual))),
    parameter(:offspring,   list(s(:individual)))
  ]

end

define_type :replacer do
  composer { s(:replacer) }
end
