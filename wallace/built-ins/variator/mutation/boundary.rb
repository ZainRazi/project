# encoding: utf-8
import('../../type/variator')

define_type :boundary_mutation do
  extends(:variator)
  composer { s(:variator, :boundary_mutation) }
end

define_specification :boundary_mutation do
  extends(:variator)

  # The per-bit probability of a mutation being performed.
  attribute :rate,  ruby(:float)

  # Constructs a new gaussian mutation operator.
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
        inputs[0][i] = rng.float > 0.5 ? representation.max : representation.min
      end
    }
    return inputs
  end

end
