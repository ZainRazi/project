# encoding: utf-8
require_relative '../namespace'

class Wallace::Namespace::Type
  include Wallace::Namespace
  include Singleton

  # Constructs the type namespace.
  def initialize
    super(:type)
  end

  # If only a single parameter is provided, then the type whose name matches
  # the supplied argument is returned. If two arguments are supplied, then a
  # given subtype of a specified subtype is returned.
  def lookup(type, subtype = nil)
    subtype.nil? ? super(:type, type) : super(type, subtype)
  end

  # Registers a given type with this namespace.
  def register(type)
    t = super(type.namespace, type.name, type)

    # If a new base type is added, create a new region within the namespace
    # to store its subtypes.
    if type.namespace == :type && !@contents.key?(type.name)
      @contents[type.name] = {}
    end

    return t
  end

end
