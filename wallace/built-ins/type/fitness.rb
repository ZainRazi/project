# encoding: utf-8
import('bootstrap')

# Slightly dodgy...
import('../fitness/best')
import('../fitness/worst')

refine_type :fitness do
  composer { s(:fitness) }
end

refine_specification :fitness do
  
  uses_module :Comparable

  dependency s(:fitness, :worst)
  dependency s(:fitness, :best)

  # Compares this fitness to another.
  method :<=>, returns: ruby(:integer), accepts: [
    parameter(:other, s(:fitness))
  ] do

    # Any fitness is better or equal to the worst fitness.
    return 1 if other.class == Fitness::Worst

    # Any fitness is worse or equal to the best fitness.
    return -1 if other.class == Fitness::Best

    # Check if the fitness object is unsupported.
    fail("Unable to compare fitness of fitness objects: #{self.class.name} and #{other.class.name}.")

  end

end
