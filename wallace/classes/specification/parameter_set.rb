# encoding: utf-8

class Wallace::Specification::ParameterSet

  # The (ordered) set of required positional arguments.
  attr_reader :required

  # The (ordered) set of optional positional arguments.
  attr_reader :optional

  # Constructs a parameter set from a list of parameters.
  #
  # ==== Parameters
  # [+parameters+]  A list of parameters.
  #
  # ==== Exceptions
  # [+Exception+] If more than one parameter may be supplied as a block.
  # [+Exception+] If an optional positional parameter is succeeded by a required
  #               positional parameter.
  def initialize(parameters = [])
    @required = []
    @optional = []
    optionals_started = false
    parameters.each { |p|
      if p.required?
        fail("Illegal parameter set: required parameters must precede optionals.") if optionals_started
        @required << p
      else
        optionals_started = true
        @optional << p
      end
    }
  end

  # Returns a list of all the parameters within this list.
  def all
    required + optional
  end

end
