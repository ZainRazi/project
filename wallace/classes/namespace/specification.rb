# encoding: utf-8
require_relative '../namespace'

class Wallace::Namespace::Specification
  include Wallace::Namespace
  include Singleton

  # Constructs the specification namespace.
  def initialize
    super(:type)
  end

  # If only a single parameter is provided, then the base specification whose
  # name matches the supplied argument is returned. If two arguments are
  # supplied, then the first is used as the namespace of the specification.
  def lookup(type, subtype = nil)
    subtype.nil? ? super(:type, type) : super(type, subtype)
  end

  # Registers a given specification with this namespace.
  def register(spec)
    super(spec.namespace, spec.name, spec)

    # If a new base type is added, create a new region within the namespace
    # to store its subtypes.
    if spec.namespace == :type && !@contents.key?(spec.name)
      @contents[spec.name] = {}
    end
  end

end
