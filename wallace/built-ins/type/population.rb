# encoding: utf-8
import('bootstrap')
import('deme')

define_specification :population do

  # A list of the demes contained within this population.
  attribute :demes, list(s(:deme))

  # Generates a fresh population of individuals.
  method :generate, accepts: [
    parameter(:rng, s(:rng))
  ] do
    demes.each { |d| d.generate(rng) }
  end

  # Returns the best individual from this population.
  method :best, returns: s(:individual) do
    best = nil
    demes.each do |deme|
      deme_best = deme.best
      best = deme_best if !deme_best.nil? && deme_best > best
    end
    return best
  end

  # Returns the individual with the highest fitness from this population.
  # method :max, returns: s(:individual) do
  #   max = nil
  #   demes.each do |deme|
  #     deme_max = deme.max
  #     max = deme_max if deme_max.evaluated && (max.nil? || deme_max > max)
  #   end
  #   return max
  # end

  # Returns the individual with the lowest fitness from this population.
  # method :min, returns: s(:individual) do
  #   min = nil
  #   demes.each do |deme|
  #     deme_min = deme.min
  #     min = deme_min if deme_min.evaluated && (min.nil? || deme_min < min)
  #   end
  #   return min
  # end
  
end

define_type :population do

  # A population is composed of a number of independently evolving demes.
  collection :demes, :deme, i(:deme)

  composer do |s_individual|
    s = spec(:population).extend
    
    # Build the constructor and add dependencies for each deme.
    s.constructor([], source: demes.map { |deme|
      si_deme = deme.compose(s_individual)
      s.dependency(si_deme.specification)
      si_deme.to_s
    }.join(', ').concat(']').prepend('self.demes = ['))

    s
  end

end
