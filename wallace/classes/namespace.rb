# encoding: utf-8

module Wallace::Namespace

  # Imports the class methods into the including class.
  def self.included(base)
    base.extend(ClassMethods)
  end

  # Constructs this namespace.
  #
  # ==== Parameters
  # [+default+] The name of the default namespace.
  def initialize(default)
    @contents = {}
    @contents[default] = {}
    @default = default
  end

  # Returns true if a given slot is occupied within the namespace.
  def has?(area, name)
    @contents.key?(area) && @contents[area].key?(name)
  end

  # Returns the item at a given slot within this namespace.
  #
  # ==== Parameters
  # [+area+]  The area of the namespace to search within.
  # [+name+]  The name of the item to search for.
  # 
  # ==== Exceptions
  # [+Exception+] If there is no item at the specified slot.
  def lookup(area, name)
    unless has?(area, name)
      fail("Failed to find item within namespace at (#{area}, #{name}).")
    end
    return @contents[area][name]
  end

  # Registers an entity within this namespace at a specified slot and area.
  #
  # ==== Parameters
  # [+area+]    The area of the namespace to insert the entity at.
  # [+name+]    The name of the slot to insert the entity at.
  # [+entity+]  The entity which should be inserted into the namespace.
  #
  # ==== Returns
  # The registered entity.
  #
  # ==== Exceptions
  # [+Exception+] If the specified area does not exist.
  def register(area, name, entity)
    unless @contents.key?(area)
      fail("Failed to register entity with namespace: no such area '#{area}' exists.")
    end
    return @contents[area][name] = entity
  end

  # Class methods are used to provide easy direct access to namespace methods,
  # rather than having to call them from the retrieved singleton instance.
  module ClassMethods
    def has?(*args, &blk); instance.has?(*args, &blk); end
    def register(*args, &blk); instance.register(*args, &blk); end
    def lookup(*args, &blk); instance.lookup(*args, &blk); end
  end

end
