# encoding: utf-8

class String

  def indent(spaces = 2)
    self.lines.map { |line| (" " * spaces) + line }.join
  end

end
