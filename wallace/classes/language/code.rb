# encoding: utf-8

class Wallace::Code

  def self.[](code)
    self.new(code)
  end

  def initialize(code)
    @code = code
  end

  def inspect
    @code
  end

end
