# encoding: utf-8
import('../type/replacer')

# The generational replacer removes all the current members of the deme, except
# for a number of the best individuals, and replaces them with their offspring.
# If there are more offspring than their are individuals within the deme then
# any excess offspring are discarded.
register s(:generational) {
  extends(:replacer)

  # The number of best individuals to retain within the deme.
  attribute :elitism, ruby(:integer)

  # Constructs a new generational replacer.
  constructor [
    parameter(:elitism, ruby(:integer), 0)
  ] do
    self.elitism = elitism
  end

  # Calculates the surviving individuals from the union of the multi-set of 
  # individuals already in the population, and the newly created offspring.
  implement_method :survivors do

    # Retain the elite individuals in the "highest" slots of the deme.
    if elitism > 0
      members[0...elitism] = (members.sort)[0...elitism]
    end 

    # Insert the truncated offspring into the remaining slots.
    members[elitism..-1] = offspring[0, members.length - elitism]

    return members

  end

}

register t(:generational) {
  extends(:replacer)
  composer { s(:replacer, :generational) }
}
