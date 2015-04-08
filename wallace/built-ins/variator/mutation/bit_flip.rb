# encoding: utf-8
import('../../type/variator')

define_type :bit_flip_mutation do
  extends(:variator)
  composer { s(:variator, :bit_flip_mutation) }
end

define_specification :bit_flip_mutation do
  extends(:variator)

  # The per-bit probability of a mutation being performed.
  attribute :rate, ruby(:float)

  # Constructs a new bit flip mutation operator.
  constructor [
    parameter(:name,    ruby(:string)),
    parameter(:rate,    ruby(:float)),
    parameter(:source,  ruby(:*))
  ] do
    super(name, source, 1)
    self.rate = rate
  end

  implement_method :operate do
    (0 ... inputs[0].length).each { |i|
      if rng.float <= rate
        inputs[0][i] = (inputs[0][i] == 0 ? 1 : 0)
      end
    }
    return inputs
  end

end
