# encoding: utf-8
class Wallace::Specification

  # Include each of the specification component classes.
  require_relative 'specification/template_parameter'
  require_relative 'specification/attribute'
  require_relative 'specification/method'
  require_relative 'specification/constructor'
  require_relative 'specification/abstract_method'
  require_relative 'specification/parameter'
  require_relative 'specification/parameter_set'
  require_relative 'specification/instance'
  require_relative 'specification/template_instance'

  # Token for template parameters.
  Wallace::T = Struct.new(:name)

  # The parent of this specification.
  attr_reader :parent

  # The name of this specification (as a symbol).
  attr_reader :name

  # The namespace that this specification belongs to (as a symbol).
  attr_reader :namespace

  # A hash of attributes for this specification.
  attr_reader :attributes

  # A hash of methods for this specification.
  attr_reader :methods

  # A hash of abstract methods for this specification.
  attr_reader :abstract_methods

  # A list of the Ruby modules that this specification should include.
  attr_reader :modules

  # A list of the Ruby files upon which this specification depends.
  attr_reader :ruby_files

  # Retrieves a given specification.
  def self.[](*args, &blk)
    Wallace::Namespace::Specification.lookup(*args, &blk)
  end

  # Constructs a new specification.
  #
  # ==== Parameters
  # [+name+]  The name of this specification.
  # [+blk+]   An optional block to execute within the scope of this specification
  #           immediately upon its construction.
  def initialize(name = "S#{object_id}".to_sym, &blk)
    @name = name
    @namespace = :type
    @hard_dependencies = []
    @parent = nil
    @template_parameters = {}
    @constructor = nil
    @attributes = {}
    @methods = {}
    @abstract_methods = {}
    @modules = []
    @ruby_files = []
    instance_eval(&blk) unless blk.nil?
  end

  # Checks whether this specification has been changed since its
  # instantiation.
  def altered?
    return  !@parent.nil? ||
            !@constructor.nil? ||
            !@methods.empty? ||
            !@attributes.empty? ||
            !@abstract_methods.empty? ||
            !@template_parameters.empty? ||
            !@modules.empty? ||
            !@ruby_files.empty?
  end

  # Returns the effective constructor for this specification.
  def effective_constructor
    if @constructor.nil?
      return @parent.nil? ? nil : @parent.effective_constructor
    else
      return @constructor
    end
  end

  def get_constructor
    @constructor
  end

  # Adds a Ruby file dependency to this specification.
  def ruby_file(f)
    @ruby_files << f unless @ruby_files.include?(f)
  end

  # Returns a list of the ancestors of this specification.
  def ancestors
    @parent.nil? ? [] : ([@parent] + @parent.ancestors)
  end

  # Returns a hash of the template parameters belonging to this specification,
  # indexed by their names.
  def template_parameters
    @template_parameters.clone
  end

  # Adds a template parameter to this specification.
  #
  # ==== Parameter
  # [+p+] A template parameter, supplied as an instance of the TemplateParameter
  #       class.
  def template_parameter(name, type)
    @template_parameters[name] = TemplateParameter.new(name, type)
  end

  # Returns true if this specification is a template (i.e. it accepts
  # template parameters).
  def template?
    !@template_parameters.empty?
  end

  # Produces an instantitation of this specification template.
  #
  # ==== Parameters
  # [+params+]  The template parameters to use.
  #
  # ==== Returns
  # The constructed template instance.
  #
  # ==== Exceptions
  # [+Exception+] If this specification isn't template. 
  def instantiate(params = {})
    unless template?
      fail("Failed to instantiate template: specification must be a template.")
    end
    return TemplateInstance.new(self, params)
  end

  # Extends this specification.
  #
  # ==== Exceptions
  # [+Exception+] If this specification is a template.
  def extend(*args)

    # Construct a new specification object, using the optionally provided name.
    s = Wallace::Specification.new(*(args[0].is_a?(Symbol) ? [args[0]] : []))

    # Make the newly constructed specification extend this specification,
    # using any supplied parameter values, provided as a hash at the end
    # of the arguments list.
    s.extends(self, args[-1].is_a?(Hash) ? args[-1] : {})

    return s

  end

  # Makes this specification the sub-class of another, using an
  # optional set of template parameters.
  #
  # ==== Parameters
  # [+parent+]  The parent specification of this specification.
  # [+params+]  An optional hash of template parameters.
  #
  # ==== Exceptions
  # [+Exception+] If the supplied parent is not a valid specification.
  def extends(*args)

    # Check that this specification hasn't been altered.
    if altered?
      fail("Failed to extend specification: specification has already been altered.")
    end

    # Extract the parent specification from the list of provided arguments.
    if (args.length >= 1 &&
          ( args[0].is_a?(Wallace::Specification) ||
            args[0].is_a?(Wallace::Specification::TemplateInstance)))
      @parent = args[0]
    elsif args.length >= 2 && args[0].is_a?(Symbol) && args[1].is_a?(Symbol)
      @parent = Wallace::Specification[args[0], args[1]]
    elsif args.length >= 1 && args[0].is_a?(Symbol)
      @parent = Wallace::Specification[args[0]]
    else
      fail("Failed to extend specification: no parent specification provided.")
    end

    # If a template instance has been provided then calculate the namespace of
    # the specification using the template of the instance.
    p = @parent.is_a?(Wallace::Specification::TemplateInstance) ? @parent.template : @parent
    @namespace = p.namespace == :type ? p.name : p.namespace

    # If the provided parent is a template, then use the provided parameters
    # to instantiate it.
    if @parent.is_a?(Wallace::Specification) && @parent.template?
      @parent = TemplateInstance.new(@parent, args[-1].is_a?(Hash) ? args[-1] : {})
    end

  end

  # Adds a Ruby module to this specification.
  #
  # ==== Parameters
  # [+name+]  The name of the Ruby module (as a string or symbol).
  def uses_module(name)
    @modules << name
  end

  # Adds an attribute to this specification.
  #
  # ==== Parameters
  # [+name+]    The name of this attribute (as a symbol).
  #
  # ==== Options
  # [+type+]    The type of this attribute.
  # [+value+]   The value of this attribute.
  #
  # ==== Exceptions
  # [+Exception+] If a type is not provided.
  def attribute(name, options = {})
    @attributes[name] = Attribute.new(name, options)
  end

  # Adds a method to this specification.
  #
  # ==== Parameters
  # [+name+]    The name of this method.
  # [+options+] A hash of keyword options for construction.
  # [+&body+]   A block containing the body for this method.
  #
  # ==== Options
  # [+returns+] The type that this method returns.
  # [+accepts+] A list of parameters that this method accepts.
  def method(name, options = {}, &body)
    @methods[name] = Method.new(name, options, &body)
  end

  # Implements an abstract method defined by a parent specification.
  #
  # ==== Parameters
  # [+name+]    The name of the method to implement.
  # [+opts+]    An optional hash of keyword options.
  # [+&blk+]   An optional proc, containing the body of the method.
  #
  # ==== Options
  # [+source+]  An optional keyword for the source of the method body.
  #
  # ==== Exceptions
  # [+Exception+] If no abstract method exists with the given name within the
  #               parent of this type.
  # [+Exception+] If this type has no parent type.
  def implement_method(name, opts = {}, &blk)

    # Check that this sub-type has a parent type.
    if @parent.nil?
      fail("This specification has no parent from which to implement methods.")
    end

    # Retrieve the hash of available abstract methods.
    m = @parent.abstract_methods

    # Check that the abstract method being implemented exists.
    unless m.key?(name)
      fail("No such (unimplemented) abstract method exists: #{name}.")
    end

    # Implement and store the resulting method.
    @methods[name] = m[name].implement(opts, &blk)

  end

  # Returns a hash of (unimplemented) abstract methods declared within this
  # specification and its parent specification, indexed by their names.
  def abstract_methods
  
    # Clone the abstract methods declared within this specification.
    am = @abstract_methods.clone

    # Retrieve the abstract methods from the parent specification and filter
    # those which are implemented by this specification.
    unless @parent.nil?
      am.merge!(@parent.abstract_methods.reject { |n, m|
        @methods.key?(n) })
    end

    return am

  end

  # Adds an abstract method to this specification.
  #
  # ==== Parameters
  # [+name+]    The name of this method.
  # [+options+] A hash of keyword options for construction.
  #
  # ==== Options
  # [+returns+] The type that this method returns.
  # [+accepts+] A list of parameters that this method accepts.
  def abstract_method(name, options = {})
    @abstract_methods[name] = AbstractMethod.new(name, options)
  end

  # Defines the constructor for this (sub-)type.
  #
  # ==== Parameter
  # [+accepts+] A list of parameters accepted by this constructor.
  # [+options+] An optional hash of keyword options.
  # [+&body+]   A block containing the body for this constructor.
  def constructor(accepts = [], options = {}, &body)
    @constructor = Constructor.new(accepts, options, &body)
  end

  # Adds a hard dependency to this specification.
  #
  # ==== Parameters
  # [+s+] The specification to add as a hard dependency to this specification.
  def dependency(s)
    if s.is_a?(Wallace::Specification)
      @hard_dependencies << s
    elsif s.is_a?(Wallace::Specification::TemplateInstance)
      @hard_dependencies += s.dependencies
    else
      fail("Failed to add hard dependency to specification: illegal specification provided.")
    end
  end


  def to_camel_case(snake_case)
    snake_case.to_s.split('_').map(&:capitalize).join
  end

  # Computes and returns the class name of this specification.
  def class_name
    if @namespace == :type
      return to_camel_case(@name)
    else
      return "#{to_camel_case(@namespace)}::#{to_camel_case(@name)}"
    end
  end

  # Calculates and returns a list of types upon which this specification
  # depends.
  def dependencies
    deps = @attributes.values.inject([]) { |d, a| d += a.dependencies } +
      @methods.values.inject([]) { |d, m| d += m.dependencies } +
      @hard_dependencies
    deps << @parent if @parent.is_a?(Wallace::Specification)
    deps += @parent.dependencies if @parent.is_a?(Wallace::Specification::TemplateInstance)

    # To avoid an infinite recursion, a specification must not depend upon itself.
    deps.delete(self)

    return deps.uniq
  end

  def full_dependencies
    p = 0
    deps = [self]
    until p == deps.length
      deps += deps[p].dependencies.reject { |d| deps.include?(d) }
      p += 1
    end
    return deps
  end

  # Returns true if this specification has no parent specification.
  def base?
    @parent.nil?
  end

  # Produces an instance of this specification.
  #
  # ==== Parameters
  # [+params+]  An optional hash of parameters for the instance constructor.
  #
  # ==== Returns
  # A specification instance object.
  def instance(params = {})
    Instance.new(self, params)
  end

  # Refines this specification using instructions provided by a block.
  def refine(&blk)
    instance_eval(&blk)
  end

  # --------------------------------------------------------------------
  # Convenience methods
  # --------------------------------------------------------------------
  def parameter(*args, &blk)
    Parameter.new(*args, &blk)
  end
  alias_method :param, :parameter
  alias_method :p, :parameter

  def list(*args, &blk)
    Wallace::List.new(*args, &blk)
  end

  def map(*args, &blk)
    Wallace::Map.new(*args, &blk)
  end

  def ruby(*args)
    Wallace::RubyType[*args]
  end

  def specification(*args, &blk)
    Wallace::Specification[*args, &blk]
  end
  alias_method :spec, :specification
  alias_method :s, :specification

end
