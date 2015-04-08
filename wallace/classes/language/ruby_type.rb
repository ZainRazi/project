# encoding: utf-8

class Wallace::RubyType

  # Load the component class files.
  require_relative 'ruby_type/register'

  # The name of this Ruby type.
  attr_reader :name

  # The class that this Ruby type uses.
  attr_reader :cls

  # Retrieves a named Ruby type from the register.
  def self.[](name)
    Register.retrieve(name)
  end

  def initialize(name, cls)
    @name = name
    @cls = cls
  end

end