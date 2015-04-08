# encoding: utf-8
import('../../type/variator')

define_type :uniform_mutation do
  extends(:variator)
  composer { s(:variator, :uniform_mutation) }
end

define_specification :uniform_mutation do
  extends(:variator)

  # The per-bit probability of a mutation being performed.
  attribute :rate, ruby(:float)

  # Constructs a new uniform mutation operator.
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
        inputs[0][i] = representation.sample_gene(rng)
      end
    }
    return inputs
  end

end
