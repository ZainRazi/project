# encoding: utf-8
class Entity

  def self.[](*args)

    # Check that all template parameters have been supplied.
    if args.length != @template_parameters.length
      fail("Failed to instantiate template: incorrect number of parameters provided.")
    end

    # Calculate the name of the requested class.
    name = args.map { |a|
      a.is_a?(Class) ? a.name.to_s.split('::').map!(&:capitalize) : a.to_s
    }.join('_').prepend('Template_')

    # Instantiate the template as a class if it hasn't been done so already.
    unless const_defined?(name)
      cls = Class.new(self)
      cls.instance_variable_set(:@template_settings, Hash[
        @template_parameters.each_with_index.map { |p, i| [p, args[i]] }])
      const_set(name, cls)
    end

    # Return the class for this template instantation.
    return const_get(name)

  end

  # Defines a template parameter for this class.
  def self.template_parameter(name)
    @template_parameters = [] if @template_parameters.nil?
    @template_parameters << name
  end

  # Returns a setting from the template of this class.
  def self.template(cls, param)
    cls = ancestors[ancestors.index(cls) - 1]
    params = cls.instance_variable_get(:@template_settings)

    # Raise an error if the requested parameter doesn't exist.
    unless params.key?(param)
      fail("No such template parameter found for #{cls.name}: #{param}")
    end

    return params[param]
  end

  # Returns a setting from the template of this class.
  def template(cls, param)
    self.class.template(cls, param)
  end

end
