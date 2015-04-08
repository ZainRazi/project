# encoding: utf-8

class Wallace::Type::Attribute

  # Import each of the attribute sub-classes.
  require_relative 'attribute/ruby'
  require_relative 'attribute/type'
  require_relative 'attribute/instance'

  # The name of this attribute (as a symbol).
  attr_reader :name

  # The value held by this attribute.
  attr_reader :value

  # Constructs a new attribute.
  def initialize(name, type, value = nil)
    @name = name
    @type = type
    @value = value
  end

  # Directly sets the value of this attribute.
  def set(value)
    @value = value
  end

end
