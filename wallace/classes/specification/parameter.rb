# encoding: utf-8
class Wallace::Specification::Parameter

  # The name of this parameter (as a symbol).
  attr_reader :name

  # The type of this parameter.
  attr_reader :type

  # The default value for this parameter.
  attr_reader :default

  # Constructs a new parameter.
  #
  # ==== Parameters
  # [+name+]      The name of this parameter (as a symbol).
  # [+type+]      The type of this parameter.
  # [+default+]   The default value for this parameter.
  def initialize(name, type, default = nil)
    @name = name
    @type = type
    @default = default
  end

  # Returns true if this parameter is required.
  def required?
    @default.nil?
  end

  # Returns true if this parameter is optional.
  def optional?
    !required?
  end

  # Returns a list of the specifications upon which this parameter depends.
  def dependencies
    return [type] if type.is_a?(Wallace::Specification)
    return type.dependencies if type.is_a?(Wallace::List) || type.is_a?(Wallace::Map)
    return []
  end

end
