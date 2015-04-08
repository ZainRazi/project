# encoding: utf-8
require_relative '../type'
require_relative 'helpers'
require_relative 'instance'

class Wallace::Type::Collection
  include Wallace::Type::Helpers
  include Enumerable

  # Import each of the types of collection.
  require_relative 'collection/enumerated'
  require_relative 'collection/indexed'

  # The name of this collection.
  attr_reader :name

  # The singular form of items within this collection.
  attr_reader :singular

  # The type of items contained within this collection.
  attr_reader :of

  # Constructs a new collection.
  #
  # ==== Parameters
  # [+name+]      The name of this collection.
  # [+singular+]  The singular form of items within this collection.
  # [+of+]        The type of items contained within this collection.
  def initialize(name, singular, of)

    # Check the class of the type of items contained within this collection.
    unless [
      Wallace::Type,
      Wallace::Type::Instance::Request,
      Wallace::RubyType
    ].include?(of.class)
      fail("Failed to create collection: type class not supported '#{of.class.name}'.")
    end

    @name = name
    @singular = singular
    @of = of

  end

  # Adds an item into this collection.
  #
  # ==== Parameters
  # [+*args+] A depickled list of arguments to this method.
  # [+&blk+]  An optional block supplied to this method.
  #
  # ==== Returns
  # The built item, ready for insertion.
  def add(*args, &blk)
    if of.is_a?(Wallace::Type)
      return compose_type(of, *args, &blk)
    elsif of.is_a?(Wallace::Type::Instance::Request)
      return compose_type_instance(of.type, *args, &blk)
    elsif of.is_a?(Wallace::RubyType)
      return compose_ruby_type(of, *args, &blk)
    end
  end

  # Iterates over the contents of this collection.
  def each
    @contents.each { |i| yield i }
  end

  # Iterates over the contents of this collection, yielding both the index
  # and value of each entry.
  def each_with_index
    @contents.each_with_index { |k, v| yield [k, v] }
  end

  # Retrieves an item from this collection by its index.
  def [](index)
    @contents[index]
  end

  # Checks whether a given item exists within this collection.
  def has?(value)
    @contents.has?(value)
  end

  # Clears all items from this collection.
  def clear
    @contents.clear
  end

  # Returns the number of items within this collection.
  def length
    @contents.length
  end
  alias_method :size, :length

  # Returns true if this collection contains no items.
  def empty?
    @contents.empty?
  end

end
