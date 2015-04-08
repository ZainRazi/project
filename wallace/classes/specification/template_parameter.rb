# encoding: utf-8
class Wallace::Specification::TemplateParameter

  # The name of this template parameter.
  attr_reader :name

  # The type of this template parameter.
  attr_reader :type

  # Convenience method for constructing template parameters.
  def self.[](*args, &blk)
    new(*args, &blk)
  end

  # Constructs a new template parameter.
  #
  # ==== Parameters
  # [+name+]  The name of this template parameter.
  # [+type+]  The type of this template parameter.
  def initialize(name, type)

    # Ensure that the type provided is a specification or a Ruby type.
    # We can add some clever symbol conversion into the language
    # at a later date.
    unless type.is_a?(Wallace::Specification) ||
      type.is_a?(Wallace::RubyType)
      fail("Failed to construct template parameter: unrecognised parameter type; should be a specification or a Ruby type.")
    end

    @name = name
    @type = type

  end

end
