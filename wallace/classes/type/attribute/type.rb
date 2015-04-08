# encoding: utf-8
require_relative '../helpers'

class Wallace::Type::TypeAttribute < Wallace::Type::Attribute
  include Wallace::Type::Helpers

  # Constructs a new type attribute.
  def initialize(name, type)
    super(name, type, type.template? ? nil : type.clone)
  end

  # Routes a request sent to this attribute.
  def route(*args, &blk)
    if args.empty? && blk.nil?
      return @value
    elsif !args.empty? || @type.template_uses_block?
      return @value = compose_type(@type, *args, &blk)
    elsif !blk.nil?
      return refine(&blk)
    end
  end

  # Refines the value held by this attribute.
  def refine(&blk)
    @value.refine(&blk)
    return self
  end

end
