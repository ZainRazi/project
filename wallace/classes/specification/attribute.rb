# encoding: utf-8

# - Fixed attributes?
class Wallace::Specification::Attribute

  # The name of this attribute (as a symbol).
  attr_reader :name

  # The type of this attribute.
  attr_reader :type

  # The current value of this attribute.
  attr_accessor :value

  # Constructs a new attribute.
  #
  # ==== Parameters
  # [+name+]    The name of this attribute (as a symbol).
  # [+type+]    The type of this attribute.
  #
  # ==== Exceptions
  # [+Exception+] If an illegal type is provided.
  def initialize(name, type)

    # Check the type of the attribute.
    unless  type.is_a?(Wallace::Specification) ||
            type.is_a?(Wallace::RubyType) ||
            type.is_a?(Wallace::List) ||
            type.is_a?(Wallace::Map) ||
            type.is_a?(Wallace::T)
      fail("Failed to create attribute: unsupported type class provided '#{type.class.name}'.")
    end

    @name = name
    @type = type
  end

  # Returns a list of the specifications upon which this attribute depends.
  def dependencies
    if type.is_a?(Wallace::Specification)
      return [type]
    elsif [ Wallace::List,
            Wallace::Map,
            Wallace::Specification::TemplateInstance
          ].include?(type.class)
      return type.dependencies
    else
      return []
    end
  end

end
