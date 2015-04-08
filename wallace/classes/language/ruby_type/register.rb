# encoding: utf-8
class Wallace::RubyType::Register
  include Singleton

  def self.store(*args); instance.store(*args); end
  def self.retrieve(*args); instance.retrieve(*args); end

  def initialize
    @contents = {}
  end

  def store(rt)
    @contents[rt.name] = rt
  end

  def retrieve(name)
    unless @contents.key?(name)
      fail("Failed to find Ruby type '#{name}'.")
    end
    return @contents[name]
  end

end
