# encoding: utf-8
require_relative '../language/ruby_type'
require_relative '../type'
require_relative '../specification'
require_relative 'instance'

module Wallace::Type::Helpers

  def ruby(*args)
    Wallace::RubyType[*args]
  end

  def type(*args)
    Wallace::Type[*args]
  end
  alias_method :t, :type

  def specification(*args)
    Wallace::Specification[*args]
  end
  alias_method :s, :specification
  alias_method :spec, :specification

  def instance(*args)
    Wallace::Type::Instance::Request.new(*args)
  end
  alias_method :i, :instance
  alias_method :inst, :instance

  def compose_ruby_type(rt, *args, &blk)
    if args.length == 1 && rt.cls.is_a?(Class) && args[0].is_a?(rt.cls)
      return args[0]
    else
      return rt.cls.new(*args, &blk)
    end
  end

  def compose_type(t, *args, &blk)

    # Supplied type object.
    if args.length == 1 && args[0].is_a?(Wallace::Type)
      t = args.shift

    # Supplied a sub-type name.
    elsif args.length >= 1 && args[0].is_a?(Symbol)
      t = compose_type_with_subtype(t, args.shift)
    end

    # If this type is a template, pass the block and any template
    # parameters to its template function to return the finished
    # type.
    if t.template?
      t_params = args[0].is_a?(Array) ? args[0] : []
      t = compose_type_with_template(t, t_params, &blk)
    

    # If the type isn't a template then use the block as a refinement.
    elsif !blk.nil?
      t.refine(&blk)
    end

    return t

  end

  def compose_type_with_subtype(t, st)
    type(t.namespace == :type ? t.name : t.namespace, st)
  end

  def compose_type_with_block(t, &blk)
    blk.nil? ? t : t.refine(&blk)
  end

  def compose_type_with_template(t, params, &blk)
    unless t.template?
      fail("Failed to compose type with template: expected template parameters.")
    end
    return t.extend(params, &blk)
  end


  def compose_type_instance(t, *args, &blk)

    # If a type instance is provided, then simply return it.
    if args.length == 1 && args[0].is_a?(Wallace::Type::Instance)
      return args[0]
    end

    # Extract and remove the instance parameters from the list of arguments.
    params = args[-1].is_a?(Hash) ? args.pop : {}

    # Compose the type from the list of arguments.
    unless args.empty?
      t = compose_type(t, *args, &nil)
    end

    # SHOULD WE CLONE THE TYPE HERE?

    # If a block has been provided, refine the type.
    # ----------------------------------------------
    # DANGEROUS!!
    # ----------------------------------------------
    t = t.refine(&blk) unless blk.nil?

    # Construct the type instance object.
    return Wallace::Type::Instance.new(t, params)

  end

end
