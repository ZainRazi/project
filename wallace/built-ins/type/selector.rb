# encoding: utf-8
import('bootstrap')
import('individual')
import('rng')
import('population')
import('statistics')

define_specification :selector do

  # The unique name of this selector.
  attribute :name, ruby(:string)

  # Constructs a new selector with a given name.
  constructor [
    parameter(:name, ruby(:string))
  ] do
    self.name = name
  end

  # Prepares a list of candidates for use with this selector. By default this
  # method returns the original set of candidates (i.e. it performs no
  # pre-processing).
  #
  # Pre-processing can be performed on this list to improve performance.
  # The list of candidates is not stored by the selector, instead it is stored
  # by an associated input node.
  #
  # ==== Parameters
  # [+random+]      The random number generator to use.
  # [+statistics+]  The current search statistics.
  # [+candidates+]  The list of candidates to select from.
  #
  # ==== Returns
  # A prepared list of candidates for use with this selector.
  abstract_method :prepare, returns: list(s(:individual)), accepts: [
    parameter(:rng,         s(:rng)),
    #parameter(:population,  s(:population)),
    parameter(:statistics,  s(:statistics)),
    parameter(:candidates,  list(s(:individual)))
  ]

  # Selects and returns a specified number of individuals from a (prepared)
  # list of candidates according to the rules of this selection method.
  abstract_method :select, returns: list(s(:individual)), accepts: [
    parameter(:rng,           s(:rng)),
    #parameter(:population,    s(:population)),
    parameter(:statistics,    s(:statistics)),
    parameter(:candidates,    list(s(:individual))),
    parameter(:num_requested, ruby(:integer))
  ]

  method :produce, returns: list(s(:individual)), accepts: [
    parameter(:rng,           s(:rng)),
    #parameter(:population,    s(:population)),
    parameter(:statistics,    s(:statistics)),
    parameter(:buffer,        map(ruby(:string), list(s(:individual)))),
    parameter(:num_requested, ruby(:integer))
  ] do
    select( rng,
        #population,
        statistics,
        buffer[name],
        num_requested).map { |i| i.clone }
  end

end

define_type :selector do
  composer { s(:selector) }
end
