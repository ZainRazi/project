# encoding: utf-8

class Wallace::Type::InstanceAttribute < Wallace::Type::Attribute
  include Wallace::Type::Helpers

  def route(*args, &blk)
    return @value if args.empty? && blk.nil?
    return @value = compose_type_instance(@type, *args, &blk) if !args.empty?
    #return refine(&blk) if !blk.nil?
  end

end
