# encoding: utf-8

class Wallace::Specification::Constructor < Wallace::Specification::Method

  # Constructs a new constructor.
  #
  # ==== Parameter
  # [+accepts+] A list of parameters accepted by this constructor.
  # [+options+] A hash of keyword options for construction.
  # [+&proc+]   An optional block containing the body for this method; must be
  #             provided if the body of the method isn't supplied by the source
  #             keyword option.
  #
  # ==== Options
  # [+source+]  The source code for the body of this constructor.
  def initialize(accepts = [], options = {}, &proc)
    super(:initialize,
      accepts: accepts,
      returns: nil,
      source: options[:source],
      &proc)
  end

  # Returns a short string description of this constructor.
  def description
    "+ initialize(#{@accepts.description})"
  end

end
