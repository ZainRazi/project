# encoding: utf-8

class Wallace::Map < Hash

  attr_reader :from
  attr_reader :of

  def self.[](from, to)
    self.new(from, to)
  end

  def initialize(from, to)
    [from, to].each do |x|
      unless [  Wallace::RubyType,
                Wallace::Specification,
                Wallace::T,
                Wallace::List,
                Wallace::Map].include?(x.class)
        fail("Failed to create map: can only hold ruby types, specifications, and template parameters.")
      end
    end

    @from = from
    @to = to
  end

  # Returns a list of the specifications upon which this map depends.
  def dependencies
    [@from, @to].inject([]) { |d, x|
      if x.is_a?(Wallace::Specification)
        d << x
      elsif x.is_a?(Wallace::List) || x.is_a?(Wallace::Map)
        d += x.dependencies
      end
      d
    }
  end

end
