# encoding: utf-8
require_relative '../specification'
require_relative '../language/list'
require_relative '../language/map'

class Wallace::Specification::TemplateInstance

  # Constructs a new template instance.
  #
  # ==== Parameters
  # [+template+]    The template specification this is an instance of.
  # [+parameters+]  
  def initialize(template, parameters = {})
    @template = template
    @parameters = parameters
  end

  # Returns a list of the specifications upon which this template instance
  # depends.
  def dependencies
    @parameters.values.inject([@template]) do |deps, p|
      case p
      when Wallace::Specification
        deps << p
      when Wallace::List
        deps += p.dependencies
      when Wallace::Map
        deps += p.dependencies
      when Wallace::Specification::TemplateInstance
        deps += p.dependencies
      end
      deps
    end
  end

  def full_dependencies
    dependencies.inject([]) { |deps, d| deps += d.full_dependencies }.uniq
  end

  # NOT SURE ABOUT THIS...
  #alias_method :full_dependencies, :dependencies

  # Returns a shallow copy of the hash of parameters used by this template
  # instance.
  def parameters
    @parameters.clone
  end

  # Returns the template of this instance.
  def parent
    @template
  end
  alias_method :template, :parent

  # Computes the class name of this template instance.
  def class_name
    @template.class_name + (parent.template_parameters.keys.map { |k|
      v = @parameters[k]
      if v.is_a?(Wallace::Specification) || v.is_a?(Wallace::Specification::TemplateInstance)
        v = v.class_name
      elsif v.is_a?(Wallace::RubyType)
        v = v.cls.name
      else
        v = v.inspect
      end
      v
    }.join(', ').prepend('[').concat(']'))
  end

  # Returns the effective constructor for this specification.
  def effective_constructor
    template.effective_constructor
  end

  # Returns a hash of the abstract methods for the template of this instance.
  def abstract_methods
    template.abstract_methods
  end

  # Produces an instance of this specification using a hash of supplied
  # parameter values.
  def instance(params = {})
    Instance.new(self, params)
  end

  # Returns a list of the ancestors of this template instance. Note that this
  # does not include the template that this is an instance of (as it does not
  # extend that template).
  def ancestors
    @template.ancestors
  end

end
