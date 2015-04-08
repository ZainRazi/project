# encoding: utf-8

class Wallace::Specification::AbstractMethod

  # The name of this abstract method.
  attr_reader :name

  # Constructs a new abstract method.
  #
  # ==== Parameters
  # [+name+]    The name of this method.
  # [+options+] A hash of keyword options for construction.
  #
  # ==== Options
  # [+returns+] The type that this method returns.
  # [+accepts+] A list of parameters that this method accepts.
  def initialize(name, options = {})
    @name = name
    @accepts = Wallace::Specification::ParameterSet.new(options[:accepts] || [])
    @returns = options[:returns] || nil
  end

  # Implements this abstract method using a given method body, returning a
  # completed Method object.
  #
  # ==== Parameters
  # [+opts+]    An optional hash of keyword options to this method.
  # [+&body+]   The body of this method as a block.
  #
  # ==== Options
  # [+source+]  The source code for this method.
  def implement(opts = {}, &body)
    if body.nil?
      Wallace::Specification::Method.new(@name,
        accepts: @accepts, returns: @returns, source: opts[:source])
    else
      Wallace::Specification::Method.new(@name,
        accepts: @accepts, returns: @returns, &body)
    end
  end

  # Returns a short string description of this method.
  def description
    "+ #{@name}(#{@accepts.description}) : #{@returns}"
  end

end
