# encoding: utf-8

class Wallace::Type::Composer

  # Constructs a new composer.
  #
  # ==== Parameters
  # [+owner+] The type that this composer belongs to.
  # [+&f+]    A block responsible for carrying out the composition.
  def initialize(owner, &f)
    @owner = owner
    @f = f
  end

  # Returns the parent composer of this composer, or nil if it has none.
  def parent
    p = @owner
    while (p = p.parent) 
      return p.composer unless p.composer.nil?
    end
    return nil
  end

  def compose(type, *args)

    # Backup any existing "super" command attached to this type (by an
    # already called child composer).
    if type.singleton_methods(false).include?(:sup)
      backup = type.singleton_class.instance_method(:sup)
    else
      backup = nil
    end

    # Attach the "super" command within the composer block.
    p = parent
    type.define_singleton_method(:sup, &lambda { |*args|
      if p.nil?
        fail("Failed to compose type: no parent composer available to call.")
      else
        p.compose(self, *args)
      end
    })

    # Execute the composition block within the context of the provided type.
    s = type.instance_exec(*args, &@f)

    # Restore the existing "super" command, or remove it entirely.
    if backup.nil?
      type.singleton_class.send(:undef_method, :sup)
    else
      backup.bind(type)
    end

    # Return the result of the compose statement.
    return s

  end

end

