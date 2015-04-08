# encoding: utf-8

class Wallace::List < Array

  # Returns the type of item contained by this list.
  attr_reader :of

  # Constructs a new typed variable length list.
  def self.[](of)
    self.new(of)
  end

  # Constructs a new typed variable length list.
  def initialize(of)
    unless [  Wallace::RubyType,
              Wallace::Specification,
              Wallace::T,
              Wallace::List,
              Wallace::Map].include?(of.class)
      fail("Failed to create list: can only hold ruby types, specifications, and template parameters.")
    end
    @of = of
  end

  # Returns a list of the types upon which this list depends.
  def dependencies
    if of.is_a?(Wallace::Specification)
      return [of]
    elsif of.is_a?(Wallace::List) || of.is_a?(Wallace::Map)
      return of.dependencies
    else
      return []
    end
  end

end
