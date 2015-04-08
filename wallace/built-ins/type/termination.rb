# encoding: utf-8
import('bootstrap')
import('statistics')
import('criterion')

define_specification :termination do

  template_parameter :statistics, s(:statistics)

  # A map of the termination criteria for this algorithm, indexed by their identifiers.
  attribute :criteria, map(ruby(:string), s(:criterion))

  # Checks whether a criterion with a given identifier exists for this algorithm.
  # WARNING - Breaks CPP compatibility.
  method :criterion?, returns: ruby(:boolean), accepts: [
    parameter(:name, ruby(:string))
  ] do
    return criteria.key?(name)
  end

  # Determines whether any of the termination conditions have been met.
  method :satisfied?, returns: ruby(:boolean), accepts: [
    parameter(:rng,         s(:rng)),
    parameter(:statistics,  T[:statistics]),
    parameter(:population,  s(:population))
  ] do
    criteria.each do |n, c|
      return true if c.satisfied?(rng, statistics, population)
    end
    return false
  end

end

define_type :termination do

  # An indexed collection of the criteria under which this algorithm
  # should terminate.
  indexed_collection :criteria, :criterion, i(:criterion)

  composer do |opts|
  
    # Construct the base termination class.
    s = s(:termination).extend(statistics: opts[:statistics])

    # Add each of the termination criteria as an attribute of this specification
    # and instantiate them each with the provided parameters in the constructor.
    cons = []
    criteria.each_pair do |n, c|
      si_c = c.compose
      s.attribute(n, si_c.specification)
      cons << "self.#{n} = #{si_c}"
    end
    cons << ""
    cons << "self.criteria = {"
    cons << criteria.keys.map { |n| "  '#{n}' => self.#{n}"}.join(",\n")
    cons << "}"

    # Build the constructor.
    s.constructor([], source: cons.join("\n"))

    s
    
  end

end
