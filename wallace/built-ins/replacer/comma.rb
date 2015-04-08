# encoding: utf-8
import('../type/replacer')

# The Comma replacer uses the n-best offspring from the current generation
# as the individuals of the next generation (where n is the equal to the
# size of the current population).
register s(:comma) {
  extends(:replacer)

  # Calculates the surviving individuals from the union of the multi-set of 
  # individuals already in the population, and the newly created offspring.
  implement_method :survivors do
    offspring.sort[0, members.length]
  end

}

register t(:comma) { 
  extends(:replacer)
  composer { s(:replacer, :comma) }
}
