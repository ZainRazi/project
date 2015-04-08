# encoding: utf-8

class Wallace::Type::IndexedCollection < Wallace::Type::Collection

  # Constructs a new indexed type collection.
  #
  # ==== Parameters
  # [+name+]      The name of this collection.
  # [+singular+]  The singular form of items within this collection.
  # [+of+]        The type of items contained within this collection.
  def initialize(name, singular, of)
    @contents = {}
    super(name, singular, of)
  end

  # Checks whether a given key is in use by this collection.
  def key?(key)
    @contents.key?(key)
  end

  # Adds an item into this collection with an associated key.
  #
  # ==== Parameters
  # [+key+]   The key to use when storing this item.
  # [+*args+] A depickled list of arguments to this method.
  # [+&blk+]  An optional block supplied to this method.
  #
  # ==== Returns
  # The inserted item.
  def add(key, *args, &blk)
    @contents[key] = super(*args, &blk)
  end

  # Removes an item from this collection by its key.
  #
  # ==== Parameters
  # [+key+] The key of the item to remove from the collection.
  #
  # ==== Returns
  # The removed item.
  def remove(key)
    @contents.delete(key)
  end

  # Iterates over each key-value pair in this indexed collection.
  def each_pair
    @contents.each_pair { |k,v| yield k, v }
  end

  # Returns a list of the keys within this indexed collection,
  # omitting their values.
  def keys
    @contents.keys
  end

  # Returns a list of the items within this indexed collection,
  # omitting their keys.
  def values
    @contents.values
  end

  # Returns a list of each of the key-value pairs within this
  # indexed collection.
  def pairs
    @contents.to_a
  end
  alias_method :to_a, :pairs

end
