# encoding: utf-8
import('int_vector')

define_type :bit_vector do
  extends(:representation, :int_vector)

  # Injects each of the conversion operations into the representation
  # and returns the complete specification.
  composer do

    # This is interesting...
    s(:representation, :bit_vector)

  end
end

define_specification :bit_vector do
  extends(:representation, :int_vector)

  # Constructs a new bit vector representation.
  constructor [
    parameter(:length, ruby(:integer))
  ] do
    super(length, 0, 1)
  end

end
