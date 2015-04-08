# encoding: utf-8

class Wallace::Specification::Instance

  attr_reader :specification

  # Constructs a new specification instance.
  #
  # ==== Parameters
  # [+specification+] The specification that this instance belongs to.
  # [+parameters+]    The parameters of this instance.
  def initialize(specification, parameters)
    @specification = specification
    @parameters = parameters
  end

  def get(n)
    @parameters[n]
  end

  def set(n, v)
    @parameters[n] = v
  end

  def to_s
    if specification.effective_constructor.nil?
      p = ""
    else
      p = specification.effective_constructor.accepts
    p = (p.required.map { |p|
      unless @parameters.key?(p.name)
        fail("Failed to compile instance: required parameter missing '#{p.name}'.")
      end
      @parameters[p.name].inspect
    } + p.optional.map { |p|
      (@parameters[p.name] || p.default).inspect
    }).join(', ')
    end
    return "#{specification.class_name}.new(#{p})"
  end

end
