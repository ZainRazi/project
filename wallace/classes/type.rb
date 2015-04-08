# encoding: utf-8
class Wallace::Type

  # Import each of the component classes.
  require_relative 'type/attribute'
  require_relative 'type/instance'
  require_relative 'type/helpers'
  require_relative 'type/collection'
  require_relative 'type/composer'

  # Include each of the modules.
  include Helpers

  # The unique name of this type.
  attr_reader :name

  # The namespace which this type belongs to.
  attr_reader :namespace

  # The parent of this type.
  attr_reader :parent

  # Retrieves a given type.
  def self.[](*args)
    Wallace::Namespace::Type.lookup(*args)
  end

  # Constructs a new type.
  #
  # ==== Parameters
  # [+name+]    The name of this type.
  # [+&blk+]    A block containing instructions on how to construct this type.
  def initialize(name = "T#{object_id}".to_sym, &blk)
    @name = name
    @namespace = :type
    @parent = nil
    @composer = nil
    @attributes = {}
    @collections = {}
    @collection_singulars = {}
    @template = nil

    instance_eval(&blk) unless blk.nil?
  end

  # Returns true if this type is a template.
  def template?
    !@template.nil?
  end

  # Returns true if this type is a template and if it accepts a block parameter.
  def template_uses_block?
    !@template.nil? && @template.parameters.last[0] == :block
  end

  # Extends this type, returning a new child type.
  def extend(*args, &blk)
    t = args[0].is_a?(Symbol) ? Wallace::Type.new(args.shift) : Wallace::Type.new
    t.extends(self, *args, &blk)
    return t
  end

  # Extends an existing type.
  #
  # ==== Parameters
  # [+*args+] A depickled list of arguments supplied to this method call.
  # [+&blk+]  An optional block provided with this method call.
  #
  # ==== Exceptions
  # [+Exception+] If this type has already been extended.
  def extends(*args, &blk)

    # Throw an exception if a parent type has already been declared.
    unless @parent.nil?
      fail("Failed to extend type: type has already been extended.")
    end

    # Throw an exception if this type has been altered in any way.
    if altered?
      fail("Failed to extend type: type has already been altered.")
    end

    # Extract the parent type object from the provided arguments.
    parent = args
    if args.length >= 1 && args[0].is_a?(Wallace::Type)
      parent = args[0]
    elsif args.length >= 2 && args[0].is_a?(Symbol) && args[1].is_a?(Symbol)
      parent = Wallace::Type[args[0], args[1]]
    elsif args.length >= 1 && args[0].is_a?(Symbol)
      parent = Wallace::Type[args[0]]
    else
      fail("Failed to extend type: no parent type specified.")
    end

    # Perform the extension, store the parent type, and update the namespace
    # of this type.
    @parent = parent
    @attributes = Hash[
      parent.send(:instance_variable_get, :@attributes).each.map { |n, a|
        [n, a.clone]
    }]
    @collections = Hash[
      parent.send(:instance_variable_get, :@collections).each.map { |n, c|
        [n, c.clone]
    }]
    @collection_singulars = Hash[@collections.values.map { |c| [c.singular, c] }]
    @namespace = parent.namespace == :type ? parent.name : parent.namespace

    # If the parent type is a template, use the provided template parameters
    # to complete it. If no parameters have been provided, then throw an
    # exception.
    if parent.template?
      if args[-1].is_a?(Array)
        temp = parent.instance_variable_get(:@template)
        temp.call(self, *args[-1], &blk)
      else
        fail("Failed to extend type: no template parameters supplied.")
      end
    end

  end

  # Attaches a new attribute to this type with a given name.
  #
  # ==== Parameters
  # [+name+]    The name of the type-attribute.
  # [+type+]    The type of this attribute.
  #
  # ==== Exception
  # [+Exception+] If the name is reserved or already in use.
  def attribute(name, type)

    # Check that the attribute name isn't reserved or already in use by
    # another attribute.
    if respond_to?(name)
      fail("Failed to create attribute: name already in use '#{name.to_s}'.")
    end
    
    case type
    when Wallace::RubyType
      @attributes[name] = RubyAttribute.new(name, type)
    when Wallace::Type
      @attributes[name] = TypeAttribute.new(name, type)
    when Wallace::Type::Instance::Request
      @attributes[name] = InstanceAttribute.new(name, type.type)
    else
      fail("Failed to create attribute: unrecognised attribute type '#{type.class.name}'.")
    end

  end

  # Attaches a new collection to this type definition with a given name.
  #
  # ==== Parameters
  # [+name+]      The name of this type-collection (plural form).
  # [+singular+]  The singular form of an item within this collection.
  # [+type+]      The type of items contained within this type-collection.
  #
  # ==== Exceptions
  # [+Exception+] If the type-collection name is reserved or already in use.
  def collection(name, singular, type)

    # Check that neither the plural nor the singular form of this
    # collection is reserved.
    if respond_to?(name)
      fail("Failed to create collection: collection name is reserved '#{name}'.")
    elsif respond_to?(singular)
      fail("Failed to create collection: singular form is reserved '#{singular}'.")
    end

    @collections[name] =
      @collection_singulars[singular] = 
        EnumeratedCollection.new(name, singular, type)

  end

  # Attaches a new indexed collection to this type definition with a given
  # name.
  #
  # ==== Parameters
  # [+name+]      The name of this collection (plural form).
  # [+singular+]  The singular form of an item within this collection.
  # [+options+]   A hash of keyword options to this method.
  #
  # ==== Options
  # [+of+]      The type of items contained within this collection.
  #
  # ==== Exceptions
  # [+Exception+] If the collection name is reserved or already in use.
  def indexed_collection(name, singular, type)

    # Check that neither the plural nor the singular form of this
    # collection is reserved.
    if respond_to?(name)
      fail("Failed to create collection: collection name is reserved '#{name}'.")
    elsif respond_to?(singular)
      fail("Failed to create collection: singular form is reserved '#{singular}'.")
    end

    @collections[name] =
      @collection_singulars[singular] = 
        IndexedCollection.new(name, singular, type)

  end

  # Composes this type.
  #
  # ==== Parameters
  # [+*args+]   A depickled list of arguments to pass to the composer during
  #             composition.
  def compose(*args)

    # If there is a composer attached to this type, then perform the
    # composition using that; otherwise use the (effective) parent composer.
    if @composer.nil?
      return parent_composer.compose(self, *args)
    else
      return @composer.compose(self, *args)
    end

  end

  # If a block is supplied, that block is used to create the composer for this
  # type. If no arguments are supplied, then the current composer is returned.
  def composer(&blk)
    return @composer if blk.nil?
    @composer = Composer.new(self, &blk)
  end

  # Returns the (effective) composer for the parent of this type.
  def parent_composer
    p = self
    while (p = p.parent)
      return p.composer unless p.composer.nil?
    end
    return nil
  end 

  # Specifies the template for this type.
  def template(temp)
    @template = temp
  end

  # Directly sets the value of a given attribute.
  def set(attribute, value)
    @attributes[attribute].set(value)
  end

  # Constructs a new instance of this type.
  def instance(params = {})
    Instance.new(self, params)
  end

  # Returns true if this type has been specified or altered in any way.
  def altered?
    !@attributes.empty?   ||
    !@collections.empty?  ||
    !@parent.nil?         ||
    !@composer.nil?       ||
    !@template.nil?
  end

  # Creates a (deep) clone of this type.
  def clone
    t = Wallace::Type.new(name)
    t.instance_variable_set(:@parent, @parent)
    t.instance_variable_set(:@namespace, @namespace)
    t.instance_variable_set(:@composer, @composer)
    t.instance_variable_set(:@template, @template)
    t.instance_variable_set(:@attributes,
      Hash[@attributes.each.map  { |n, a| [n, a.clone] }])
    t.instance_variable_set(:@collections,
      Hash[@collections.each.map  { |n, c| [n, c.clone] }])
    t.instance_variable_set(:@collection_singulars, Hash[
      t.instance_variable_get(:@collections).values.map { |c| [c.singular, c] }])
    return t 
  end

  # Performs a specified refinement on this type, according to instructions
  # provided by a block.
  #
  # ==== Returns
  # This (now refined) type.
  def refine(&blk)
    instance_exec(&blk) unless blk.nil?
    return self
  end

  def method_missing(m, *args, &blk)

    # Collection retrieval.
    return @collections[m] if @collections.key?(m)

    # Collection insertion.
    return @collection_singulars[m].add(*args, &blk) if @collection_singulars.key?(m)

     # Attribute operations.
    return @attributes[m].route(*args, &blk) if @attributes.key?(m)

    # Default method missing behaviour.
    return super(m, *args, &blk)

  end

  def respond_to_missing?(m, include_private = false)
    @attributes.key?(m)           ||
    @collections.key?(m)          ||
    @collection_singulars.key?(m)
  end

end
