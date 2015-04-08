# encoding: utf-8

class Wallace::Type::EnumeratedCollection < Wallace::Type::Collection

  # Constructs a new collection.
  #
  # ==== Parameters
  # [+name+]      The name of this collection.
  # [+singular+]  The singular form of items within this collection.
  # [+of+]        The type of items contained within this collection.
  def initialize(name, singular, of)
    @contents = []
    super(name, singular, of)
  end

  # Adds an item into this collection.
  #
  # ==== Parameters
  # [+*args+] A depickled list of arguments to this method.
  # [+&blk+]  An optional block supplied to this method.
  def add(*args, &blk)
    @contents << super(*args, &blk)
  end

  # Returns a list of the values within this collection.
  def values
    @contents
  end

end
