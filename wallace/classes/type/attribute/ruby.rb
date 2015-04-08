# encoding: utf-8

class Wallace::Type::RubyAttribute < Wallace::Type::Attribute

  def route(*args, &blk)

    # Retrieval.
    if args.empty? && blk.nil?
      return @value

    # Set to provided value.
    elsif args.length == 1 && @type.cls.is_a?(Class) && args[0].is_a?(@type.cls)
      @value = args[0]

    # Invoke constructor with provided arguments.
    else
      @value = @type.cls.new(*args, &blk)
    end
  end

end
