# encoding: utf-8

class Wallace::Type::Instance

  class Request
    attr_reader :type
    def initialize(*args)
      @type = Wallace::Type[*args]
    end
  end

  # The type of this instance.
  attr_reader :type

  # The parameters for this instance.
  attr_reader :parameters

  # Constructs a new type instance.
  #
  # ==== Parameters
  # [+type+]        The type of this instance.
  # [+parameters+]  The parameters for this instance.
  def initialize(type, parameters)
    @type = type
    @parameters = parameters
  end

  def get(p)
    @parameters[p]
  end

  def set(p, v)
    @parameters[p] = v
  end

  # Composes this type instance into a specification instance.
  def compose(*args)
    Wallace::Specification::Instance.new(@type.compose(*args), @parameters)
  end

end
