# encoding: utf-8
import('bootstrap')
import('chromosome')
import('individual')
import('rng')
import('population')
import('statistics')
import('representation')

# BIG PROBLEM:
# - We can ignore this FOR NOW...
#   but we need to know what type of representation this variator works upon.
# - We need to introduce a base class for all operators too!

define_specification :variator do

  attribute :name,        ruby(:string)
  attribute :source,      ruby(:*) # THIS IS A BIG PROBLEM!
  attribute :num_inputs,  ruby(:integer)

  # Constructs a new variator.
  constructor [
    parameter(:name,        ruby(:string)),
    parameter(:source,      ruby(:*)),
    parameter(:num_inputs,  ruby(:integer))
  ] do
    self.name = name
    self.source = source
    self.num_inputs = num_inputs
  end

  # Performs this operation on a list of parent chromosomes, returning
  # a list of child chomosomes.
  abstract_method :operate, returns: list(s(:chromosome)), accepts: [
    parameter(:rng, s(:rng)),
    parameter(:representation, s(:representation)),
    parameter(:inputs, list(s(:chromosome)))
  ]

  method :produce, returns: list(s(:individual)), accepts: [
    parameter(:rng,           s(:rng)),
    parameter(:statistics,    s(:statistics)),
    parameter(:buffer,        map(ruby(:string), list(s(:individual)))),
    parameter(:num_requested, ruby(:integer))
  ] do
    until buffer[name].length >= num_requested
      inds = source.produce(rng,
                            statistics,
                            buffer,
                            num_inputs)
      operate(rng, inds[0].species.representation, inds.map { |i| i.genotype }).each_with_index do |g, i|
        inds[i].genotype = g
      end
      buffer[name] += inds
    end
    return buffer[name].pop(num_requested)
  end

end

define_type :variator do
  composer { s(:variator) }
end
